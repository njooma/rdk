package cli

import (
	"bytes"
	"context"
	"embed"
	"encoding/json"
	"fmt"
	"io"
	"io/fs"
	"net/http"
	"os"
	"os/exec"
	"path"
	"path/filepath"
	"regexp"
	"runtime"
	"slices"
	"strings"
	"text/template"
	"time"

	"charm.land/huh/v2"
	semver "github.com/Masterminds/semver/v3"
	"github.com/pkg/errors"
	"github.com/urfave/cli/v3"
	"go.viam.com/utils"
	"golang.org/x/text/cases"
	"golang.org/x/text/language"

	"go.viam.com/rdk/cli/module_generate/modulegen"
	gen "go.viam.com/rdk/cli/module_generate/scripts"
)

//go:embed module_generate/scripts
var scripts embed.FS

//go:embed all:module_generate/_templates/*
var templates embed.FS

const (
	version                        = "0.1.0"
	basePath                       = "module_generate"
	templatePrefix                 = "tmpl-"
	python                         = "python"
	cpp                            = "cpp"
	golang                         = "go"
	moduleVisibilityPrivate        = "private"
	moduleVisibilityPublic         = "public"
	moduleVisibilityPublicUnlisted = "public_unlisted"
	minPythonVersion               = "3.10"
	minGoVersion                   = "1.23"
)

var (
	supportedModuleGenLanguages = []string{python, golang, cpp}
	visibilityOption            = []string{moduleVisibilityPrivate, moduleVisibilityPublic, moduleVisibilityPublicUnlisted}
)

var (
	scriptsPath   = path.Join(basePath, "scripts")
	templatesPath = path.Join(basePath, "_templates")
)

type appInputs struct {
	AppName string
	AppType string
}

// appTemplateData is the struct passed to app template rendering.
type appTemplateData struct {
	ModuleName      string
	ModuleLowercase string
	AppName         string
	AppType         string
	Namespace       string
	Visibility      string
	// ConfigName is the name used for the webapp Config struct and its references in the
	// generated file. It is "Config" for a standalone generated app (the only code in the
	// package) and "WebappConfig" when adding an app to an existing module (to avoid
	// colliding with the Config already declared there).
	ConfigName string
}

// printAppBuildNextSteps prints the standard "build your frontend" instructions after an app
// is created or added to a module. If readmePath is non-empty, a final line pointing to that
// README is also printed.
func printAppBuildNextSteps(w io.Writer, readmePath string) {
	printf(w, "\n--- Build your frontend ---")
	printf(w, "Replace dist/index.html with your own frontend built with any framework (e.g. Svelte, Vue, React).")
	printf(w, "\n1. Install the Viam SDK:")
	printf(w, "     npm install @viamrobotics/sdk typescript-cookie")
	printf(w, "\n2. Build your frontend into the dist/ directory.")
	printf(w, "\n3. Build the module:")
	printf(w, "     make setup")
	printf(w, "     make")
	if readmePath != "" {
		printf(w, "\nSee %s for full details, including how to test during development and upload to viamapplications.com.", readmePath)
	}
}

// renderAppTemplate renders tmpl- prefixed files from _templates/app/ with app-specific data.
func renderAppTemplate(cmd *cli.Command, moduleName string, data appTemplateData, globalArgs globalArgs) error {
	debugf(cmd.Root().Writer, globalArgs.Debug, "Rendering app template files")
	appPath := path.Join(templatesPath, "app")
	tempDir, err := fs.Sub(templates, appPath)
	if err != nil {
		return err
	}
	return fs.WalkDir(tempDir, ".", func(filePath string, d fs.DirEntry, err error) error {
		if err != nil {
			return err
		}
		if !d.IsDir() && strings.HasPrefix(d.Name(), templatePrefix) {
			destPath := filepath.Join(moduleName, strings.ReplaceAll(filePath, templatePrefix, ""))
			debugf(cmd.Root().Writer, globalArgs.Debug, "\tRendering file %s", destPath)

			tFile, err := templates.Open(path.Join(appPath, filePath))
			if err != nil {
				return err
			}
			defer utils.UncheckedErrorFunc(tFile.Close)
			tBytes, err := io.ReadAll(tFile)
			if err != nil {
				return err
			}

			tmpl, err := template.New(filePath).Parse(string(tBytes))
			if err != nil {
				return err
			}

			//nolint:gosec
			destFile, err := os.Create(destPath)
			if err != nil {
				return err
			}
			defer utils.UncheckedErrorFunc(destFile.Close)

			if err := tmpl.Execute(destFile, data); err != nil {
				return errors.Wrapf(err, "error rendering template %s", destPath)
			}
		}
		return nil
	})
}

// buildResourceOptions builds the huh select options list from the available resources.
func buildResourceOptions() []huh.Option[string] {
	titleCaser := cases.Title(language.Und)
	resourceOptions := []huh.Option[string]{}
	for _, resource := range modulegen.Resources {
		words := strings.Split(strings.ReplaceAll(resource, "_", " "), " ")
		for i, word := range words {
			switch word {
			case "mlmodel":
				words[i] = "MLModel"
			case "slam":
				words[i] = "SLAM"
			default:
				words[i] = titleCaser.String(word)
			}
		}
		// we differentiate generic-service and generic-component in `modulegen.Resources`
		// but they still have the type listed. This carveout prevents the user prompt from
		// suggesting `Generic Component Component` or `Generic Service Service` as an option,
		// either visually or under the hood
		var resType string
		if words[0] == "Generic" {
			resType = strings.Join(words[:2], " ")
			// specific carveout to ensure that the `resource` is either `generic service` or
			// `generic component`, as opposed to `generic_service service`
			resource = strings.ToLower(resType)
		} else {
			resType = strings.Join(words, " ")
		}
		resourceOptions = append(resourceOptions, huh.NewOption(resType, resource))
	}
	return resourceOptions
}

// clarifyResolveOrgErr turns the org lookup's lower-level failures into something the user
// can act on. The previous generate flow also tried to log the user in from here, which both
// wrote to the terminal mid-render and could not trigger anyway; the wizard now signs the user
// in before anything is resolved.
func clarifyResolveOrgErr(caught error) error {
	switch {
	case strings.Contains(caught.Error(), "not logged in"),
		strings.Contains(caught.Error(), "error while refreshing token"):
		return errors.Wrap(caught, "run `viam login` and try again")
	case strings.Contains(caught.Error(), "none of your organizations have a public namespace"),
		strings.Contains(caught.Error(), "no organization found for"):
		return errors.Wrap(caught, "cannot create a project for an organization of which you are not a member")
	default:
		return caught
	}
}

func wrapResolveOrg(ctx context.Context, c *viamClient, newModule *modulegen.ModuleInputs) error {
	// If we're not registering on app, we don't need to resolve the org
	if !newModule.RegisterOnApp {
		nonAlphanumericRegex := regexp.MustCompile(`[^a-zA-Z0-9_-]+`)
		cleanNamespace := nonAlphanumericRegex.ReplaceAllString(newModule.Namespace, "")
		newModule.Namespace = cleanNamespace
		newModule.OrgID = newModule.Namespace
		return nil
	}

	uuidMatch, err := regexp.MatchString("^[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}$", newModule.Namespace)
	if !uuidMatch || err != nil {
		// If newModule.Namespace is NOT a UUID
		org, err := resolveOrg(ctx, c, newModule.Namespace, "")
		if err != nil {
			return clarifyResolveOrgErr(err)
		}
		newModule.OrgID = org.GetId()
	} else {
		// If newModule.Namespace is a UUID/OrgID
		org, err := resolveOrg(ctx, c, "", newModule.Namespace)
		if err != nil {
			return clarifyResolveOrgErr(err)
		}
		newModule.OrgID = newModule.Namespace
		newModule.Namespace = org.GetPublicNamespace()
		if newModule.Namespace == "" {
			return errors.New("cannot create module in an organization with no public namespace. Set a namespace for your organization")
		}
	}
	return nil
}

// populateAdditionalInfo fills in additional info in newModule.
func populateAdditionalInfo(newModule *modulegen.ModuleInputs) {
	newModule.GeneratedOn = time.Now().UTC()
	newModule.GeneratorVersion = version
	// TODO(RSDK-9727) - this is a bit inefficient because `newModule.Resource` is set above in
	// `generateModuleAction` based on `ResourceType` and `ResourceSubtype`, which are then
	// overwritten based on `newModule.Resource`! Unfortunately fixing this is slightly complicated
	// due to cases where a user didn't pass a `ResourceSubtype`, and so it was set in the `promptUser`
	// call. We should look into simplifying though, such that all these values are only ever set once.
	newModule.ResourceSubtype = strings.Split(newModule.Resource, " ")[0]
	newModule.ResourceSubtypeSnake = strings.Split(newModule.Resource, " ")[0]
	newModule.ResourceType = strings.Split(newModule.Resource, " ")[1]

	titleCaser := cases.Title(language.Und)
	replacer := strings.NewReplacer("_", " ", "-", " ")
	snakeReplacer := strings.NewReplacer("-", "_", " ", "_")
	spaceReplacer := modulegen.SpaceReplacer
	newModule.ModulePascal = spaceReplacer.Replace(titleCaser.String(replacer.Replace(newModule.ModuleName)))
	newModule.ModuleCamel = strings.ToLower(string(newModule.ModulePascal[0])) + newModule.ModulePascal[1:]
	newModule.ModuleLowercase = strings.ToLower(newModule.ModulePascal)
	newModule.ModuleSnake = snakeReplacer.Replace(newModule.ModuleName)
	newModule.API = fmt.Sprintf("rdk:%s:%s", newModule.ResourceType, newModule.ResourceSubtype)
	newModule.ResourceSubtypePascal = spaceReplacer.Replace(titleCaser.String(replacer.Replace(newModule.ResourceSubtype)))
	if newModule.Language == golang {
		newModule.ResourceSubtype = spaceReplacer.Replace(newModule.ResourceSubtype)
	}
	newModule.ResourceTypePascal = spaceReplacer.Replace(titleCaser.String(replacer.Replace(newModule.ResourceType)))
	newModule.ModelPascal = spaceReplacer.Replace(titleCaser.String(replacer.Replace(newModule.ModelName)))
	newModule.ModelCamel = strings.ToLower(string(newModule.ModelPascal[0])) + newModule.ModelPascal[1:]
	newModule.ModelSnake = snakeReplacer.Replace(newModule.ModelName)
	if newModule.ResourceSubtype == "switch" {
		newModule.ResourceSubtypeAlias = "sw"
	} else {
		newModule.ResourceSubtypeAlias = newModule.ResourceSubtype
	}

	modelTriple := fmt.Sprintf("%s:%s:%s", newModule.Namespace, newModule.ModuleName, newModule.ModelName)
	newModule.ModelTriple = modelTriple
	// Create separate model documentation file instead of README section
	modelDocFilename := fmt.Sprintf("%s_%s_%s.md", newModule.Namespace, newModule.ModuleName, newModule.ModelName)
	newModule.ModelReadmeLink = modelDocFilename
	newModule.ModuleReadmeLink = defaultReadmeFilename
}

// Creates a new directory with moduleName.
func setupDirectories(cmd *cli.Command, moduleName string, globalArgs globalArgs) error {
	debugf(cmd.Root().Writer, globalArgs.Debug, "Setting up directories")
	err := os.Mkdir(moduleName, 0o750)
	if err != nil {
		return err
	}
	return nil
}

func renderCommonFiles(cmd *cli.Command, module modulegen.ModuleInputs, globalArgs globalArgs) error {
	debugf(cmd.Root().Writer, globalArgs.Debug, module.ResourceSubtypePascal)
	debugf(cmd.Root().Writer, globalArgs.Debug, "Rendering common files")

	// Render .viam-gen-info
	infoBytes, err := json.MarshalIndent(module, "", "  ")
	if err != nil {
		return err
	}

	infoFilePath := filepath.Join(module.ModuleName, ".viam-gen-info")
	//nolint:gosec
	infoFile, err := os.Create(infoFilePath)
	if err != nil {
		return errors.Wrapf(err, "failed to create %s", infoFilePath)
	}
	defer utils.UncheckedErrorFunc(infoFile.Close)

	if _, err := infoFile.Write(infoBytes); err != nil {
		return errors.Wrapf(err, "failed to write generator info to %s", infoFilePath)
	}

	// Render README.md
	if err := renderReadme(module); err != nil {
		return errors.Wrap(err, "failed to render README.md")
	}

	// Render model documentation file
	if err := renderModelDoc(module); err != nil {
		return errors.Wrap(err, "failed to render model documentation")
	}

	// Render workflows for cloud build
	debugf(cmd.Root().Writer, globalArgs.Debug, "\tCreating cloud build workflow")
	destWorkflowPath := filepath.Join(module.ModuleName, ".github")
	if err = os.Mkdir(destWorkflowPath, 0o750); err != nil {
		return errors.Wrap(err, "failed to create cloud build workflow")
	}

	workflowPath := path.Join(templatesPath, ".github")
	workflowFS, err := fs.Sub(templates, workflowPath)
	if err != nil {
		return errors.Wrap(err, "failed to create cloud build workflow")
	}

	err = fs.WalkDir(workflowFS, ".", func(filePath string, d fs.DirEntry, err error) error {
		if err != nil {
			return err
		}
		if d.IsDir() {
			if d.Name() != ".github" {
				debugf(cmd.Root().Writer, globalArgs.Debug, "\t\tCopying %s directory", d.Name())
				err = os.Mkdir(filepath.Join(destWorkflowPath, filePath), 0o750)
				if err != nil {
					return err
				}
			}
		} else if !strings.HasPrefix(d.Name(), templatePrefix) {
			debugf(cmd.Root().Writer, globalArgs.Debug, "\t\tCopying file %s", filePath)
			srcFile, err := templates.Open(path.Join(workflowPath, filePath))
			if err != nil {
				return errors.Wrapf(err, "error opening file %s", srcFile)
			}
			defer utils.UncheckedErrorFunc(srcFile.Close)

			destPath := filepath.Join(destWorkflowPath, filePath)
			//nolint:gosec
			destFile, err := os.Create(destPath)
			if err != nil {
				return errors.Wrapf(err, "failed to create file %s", destPath)
			}
			defer utils.UncheckedErrorFunc(destFile.Close)

			_, err = io.Copy(destFile, srcFile)
			if err != nil {
				return errors.Wrapf(err, "error executing template for %s", destPath)
			}
		}
		return nil
	})
	if err != nil {
		return errors.Wrap(err, "failed to render all common files")
	}
	return nil
}

// copyLanguageTemplate copies the files from templates/language directory into the moduleName root directory.
func copyLanguageTemplate(cmd *cli.Command, language, moduleName string, globalArgs globalArgs) error {
	// templates are stored elsewhere for C++
	if language == cpp {
		return nil
	}
	debugf(cmd.Root().Writer, globalArgs.Debug, "Creating %s template files", language)
	languagePath := path.Join(templatesPath, language)
	tempDir, err := fs.Sub(templates, languagePath)
	if err != nil {
		return err
	}
	err = fs.WalkDir(tempDir, ".", func(filePath string, d fs.DirEntry, err error) error {
		if err != nil {
			return err
		}
		if d.IsDir() {
			if d.Name() != language {
				debugf(cmd.Root().Writer, globalArgs.Debug, "\tCopying %s directory", d.Name())
				err = os.MkdirAll(filepath.Join(moduleName, filePath), 0o750)
				if err != nil {
					return err
				}
			}
		} else if !strings.HasPrefix(d.Name(), templatePrefix) {
			// Copy .bat files for windows and .sh files for everything else
			if runtime.GOOS == osWindows && strings.HasSuffix(d.Name(), ".sh") {
				return nil
			}
			if runtime.GOOS != osWindows && strings.HasSuffix(d.Name(), ".bat") {
				return nil
			}
			debugf(cmd.Root().Writer, globalArgs.Debug, "\tCopying file %s", filePath)
			srcFile, err := templates.Open(path.Join(languagePath, filePath))
			if err != nil {
				return errors.Wrapf(err, "error opening file %s", srcFile)
			}
			defer utils.UncheckedErrorFunc(srcFile.Close)

			destPath := filepath.Join(moduleName, filePath)
			//nolint:gosec
			destFile, err := os.Create(destPath)
			if err != nil {
				return errors.Wrapf(err, "failed to create file %s", destPath)
			}
			defer utils.UncheckedErrorFunc(destFile.Close)

			_, err = io.Copy(destFile, srcFile)
			if err != nil {
				return errors.Wrapf(err, "error executing template for %s", destPath)
			}
			if filepath.Ext(destPath) == ".sh" && runtime.GOOS != osWindows {
				//nolint:gosec
				err = os.Chmod(destPath, 0o750)
				if err != nil {
					return errors.Wrapf(err, "error making file executable for %s", destPath)
				}
			}
		}
		return nil
	})
	if err != nil {
		return errors.Wrapf(err, "failed to render all %s files", language)
	}
	return nil
}

// Render all the files in the new directory.
func renderTemplate(cmd *cli.Command, module modulegen.ModuleInputs, globalArgs globalArgs) error {
	// templates are stored in separate repo for C++
	if module.Language == cpp {
		return nil
	}
	debugf(cmd.Root().Writer, globalArgs.Debug, "Rendering template files")
	languagePath := path.Join(templatesPath, module.Language)
	tempDir, err := fs.Sub(templates, languagePath)
	if err != nil {
		return err
	}
	err = fs.WalkDir(tempDir, ".", func(filePath string, d fs.DirEntry, err error) error {
		if !d.IsDir() && strings.HasPrefix(d.Name(), templatePrefix) {
			destPath := filepath.Join(module.ModuleName, strings.ReplaceAll(filePath, templatePrefix, ""))
			debugf(cmd.Root().Writer, globalArgs.Debug, "\tRendering file %s", destPath)

			tFile, err := templates.Open(path.Join(languagePath, filePath))
			if err != nil {
				return err
			}
			defer utils.UncheckedErrorFunc(tFile.Close)
			tBytes, err := io.ReadAll(tFile)
			if err != nil {
				return err
			}

			tmpl, err := template.New(filePath).Parse(string(tBytes))
			if err != nil {
				return err
			}

			//nolint:gosec
			destFile, err := os.Create(destPath)
			if err != nil {
				return err
			}
			defer utils.UncheckedErrorFunc(destFile.Close)

			err = tmpl.Execute(destFile, module)
			if err != nil {
				return err
			}
		}
		return nil
	})
	return err
}

// Generate stubs for the resource.
func generateStubs(cmd *cli.Command, module modulegen.ModuleInputs, globalArgs globalArgs) error {
	debugf(cmd.Root().Writer, globalArgs.Debug, "Generating %s stubs", module.Language)
	switch module.Language {
	case python:
		return generatePythonStubs(module)
	case golang:
		return generateGolangStubs(module)
	case cpp:
		return generateCppStubs(module)
	default:
		return errors.Errorf("cannot generate stubs for language %s", module.Language)
	}
}

func generateCppStubs(module modulegen.ModuleInputs) error {
	rendered, err := gen.RenderCppTemplates(module)
	if err != nil {
		return errors.Wrap(err, "cannot generate cpp stubs -- generator script encountered an error")
	}

	srcDir := filepath.Join(module.ModuleName, "src")
	if err = os.MkdirAll(srcDir, 0o750); err != nil {
		return errors.Wrap(err, "cannot generate cpp stubs -- unable to create src directory")
	}

	filesToWrite := []struct {
		path string
		data []byte
	}{
		{filepath.Join(module.ModuleName, "main.cpp"), rendered.Main},
		{filepath.Join(srcDir, fmt.Sprintf("%s.cpp", module.ModelSnake)), rendered.Type},
		{filepath.Join(srcDir, fmt.Sprintf("%s.hpp", module.ModelSnake)), rendered.Header},
		{filepath.Join(module.ModuleName, "CMakeLists.txt"), rendered.CMakeLists},
		{filepath.Join(module.ModuleName, "conanfile.py"), rendered.ConanFile},
		{filepath.Join(module.ModuleName, "conan.lock"), rendered.ConanLock},
	}
	for _, f := range filesToWrite {
		file, err := os.Create(f.path)
		if err != nil {
			return errors.Wrapf(err, "cannot generate cpp stubs -- unable to open %s", f.path)
		}
		defer utils.UncheckedErrorFunc(file.Close)
		if _, err = file.Write(f.data); err != nil {
			return errors.Wrapf(err, "cannot generate cpp stubs -- unable to write to %s", f.path)
		}
	}

	return nil
}

func generateGolangStubs(module modulegen.ModuleInputs) error {
	out, err := gen.RenderGoTemplates(module, false)
	if err != nil {
		return errors.Wrap(err, "cannot generate go stubs -- generator script encountered an error")
	}
	modulePath := filepath.Join(module.ModuleName, "module.go")
	//nolint:gosec
	moduleFile, err := os.Create(modulePath)
	if err != nil {
		return errors.Wrap(err, "cannot generate go stubs -- unable to open file")
	}
	defer utils.UncheckedErrorFunc(moduleFile.Close)
	_, err = moduleFile.Write(out)
	if err != nil {
		return errors.Wrap(err, "cannot generate go stubs -- unable to write to file")
	}

	// run goimports on module file out here
	err = runGoImports(moduleFile)
	if err != nil {
		return errors.Wrap(err, "cannot generate go stubs -- unable to sort imports")
	}

	// run go mod tidy
	if module.Language == golang {
		if out, err := runGoWithRetry(module.ModuleName, "mod", "tidy"); err != nil {
			return fmt.Errorf("failed to run go mod tidy: %w\n%s", err, out)
		}
	}

	return nil
}

// goFetchAttempts bounds how many times runGoWithRetry re-runs a `go` subcommand that
// failed while talking to the module proxy or checksum database.
const goFetchAttempts = 3

// goFetchRetryDelay is the base backoff between attempts; attempt N waits N times this.
const goFetchRetryDelay = 2 * time.Second

// transientGoFetchMarkers are substrings the Go toolchain emits when proxy.golang.org or
// sum.golang.org drops a request mid-flight. They say nothing about the module being
// fetched, so the same command usually succeeds on a retry. Errors that describe the
// module itself -- a bad version, a missing package, a checksum mismatch -- are absent
// here on purpose, so a genuinely broken dependency still fails on the first attempt.
var transientGoFetchMarkers = []string{
	"INTERNAL_ERROR; received from peer",
	"connection reset by peer",
	"unexpected EOF",
	"TLS handshake timeout",
	"i/o timeout",
	"502 Bad Gateway",
	"503 Service Unavailable",
	"504 Gateway Timeout",
}

// transientGoFetchFailure reports whether combined `go` output looks like a retryable
// module-fetch failure rather than a real problem with the module graph.
func transientGoFetchFailure(output []byte) bool {
	out := string(output)
	for _, marker := range transientGoFetchMarkers {
		if strings.Contains(out, marker) {
			return true
		}
	}
	return false
}

// runGoWithRetry runs a `go` subcommand in dir, retrying transient module-proxy and
// checksum-database failures. It returns the combined output of the final attempt so
// callers can surface the toolchain's own diagnostics.
func runGoWithRetry(dir string, args ...string) ([]byte, error) {
	var out []byte
	var err error
	for attempt := 1; ; attempt++ {
		//nolint: gosec,noctx
		cmd := exec.Command(golang, args...)
		cmd.Dir = dir
		out, err = cmd.CombinedOutput()
		if err == nil || attempt == goFetchAttempts || !transientGoFetchFailure(out) {
			return out, err
		}
		time.Sleep(time.Duration(attempt) * goFetchRetryDelay)
	}
}

// run goimports to remove unused imports and add necessary imports.
func runGoImports(moduleFile *os.File) error {
	// check if the gopath is set
	goPath, err := checkGoPath()
	if err != nil {
		return err
	}

	// check if goimports exists in the bin directory
	goImportsName := "goimports"
	if runtime.GOOS == osWindows {
		goImportsName = "goimports.exe"
	}
	goImportsPath := filepath.Join(goPath, "bin", goImportsName)
	if _, err := os.Stat(goImportsPath); os.IsNotExist(err) {
		// installing goimports
		if out, err := runGoWithRetry("", "install", "golang.org/x/tools/cmd/goimports@latest"); err != nil {
			return fmt.Errorf("failed to install goimports: %w\n%s", err, out)
		}
	}

	// goimports is installed. Run goimport on the module file
	//nolint: gosec,noctx
	formatCmd := exec.Command(goImportsPath, "-w", moduleFile.Name())
	_, err = formatCmd.Output()
	if err != nil {
		return fmt.Errorf("failed to run goimports: %w", err)
	}
	return err
}

func checkGoPath() (string, error) {
	//nolint: noctx
	goPathCmd := exec.Command("go", "env", "GOPATH")
	goPathBytes, err := goPathCmd.Output()
	if err != nil {
		return "", fmt.Errorf("failed to get GOPATH: %w", err)
	}
	goPath := strings.TrimSpace(string(goPathBytes))

	return goPath, err
}

func checkVersionCompatible(versionOutput, languageName, minVersion string) error {
	re := regexp.MustCompile(`(\d+\.\d+[\.\d]*)`)
	match := re.FindString(versionOutput)
	if match == "" {
		return fmt.Errorf("cannot parse %s version from output: %s", languageName, strings.TrimSpace(versionOutput))
	}
	detected, err := semver.NewVersion(match)
	if err != nil {
		return fmt.Errorf("cannot parse %s version from output: %s", languageName, strings.TrimSpace(versionOutput))
	}
	minimum := semver.MustParse(minVersion)
	if detected.LessThan(minimum) {
		return fmt.Errorf(
			"detected %s version %s, which is not supported by the module generator. Please upgrade to %s >= %s",
			languageName, detected, languageName, minVersion,
		)
	}
	return nil
}

// checkLanguageVersion checks that the system has a compatible version of the
// selected language runtime before proceeding with module generation.
func checkLanguageVersion(language string) error {
	var cmd, versionFlag, displayName, minVersion string
	switch language {
	case python:
		cmd = findPythonCommand()
		versionFlag = "--version"
		displayName = "Python"
		minVersion = minPythonVersion
	case golang:
		cmd = "go"
		versionFlag = "version"
		displayName = "Go"
		minVersion = minGoVersion
	default:
		return nil
	}
	if cmd == "" {
		return fmt.Errorf("%s runtime not found. Please install %s >= %s", displayName, displayName, minVersion)
	}
	//nolint: noctx
	versionOutput, err := exec.Command(cmd, versionFlag).Output() //nolint:gosec
	if err != nil {
		return errors.Wrapf(err, "%s runtime not found", displayName)
	}
	return checkVersionCompatible(string(versionOutput), displayName, minVersion)
}

// findPythonCommand returns the python command to use, checking "python3" then "python".
// On Windows, only "python" is checked.
func findPythonCommand() string {
	candidates := []string{"python3", "python"}
	if runtime.GOOS == osWindows {
		candidates = []string{"python"}
	}
	for _, cmd := range candidates {
		if _, err := exec.LookPath(cmd); err == nil {
			return cmd
		}
	}
	return ""
}

// createPythonVenv creates a Python virtual environment at venvName. Creating a venv
// bootstraps pip via ensurepip in a subprocess, which can fail intermittently under CI
// load, so we retry a few times and remove any partially-created environment between
// attempts. The returned error includes the subprocess's stderr, since a bare
// "exit status 1" is not actionable on its own.
func createPythonVenv(pythonCmd, venvName string) error {
	const maxAttempts = 3
	var err error
	for attempt := 0; attempt < maxAttempts; attempt++ {
		//nolint: gosec,noctx
		cmd := exec.Command(pythonCmd, "-m", "venv", venvName)
		var stderr bytes.Buffer
		cmd.Stderr = &stderr
		if err = cmd.Run(); err == nil {
			return nil
		}
		err = errorWithStderr(err, stderr.String())
		// Remove any partial environment so the next attempt starts clean.
		utils.UncheckedError(os.RemoveAll(venvName))
	}
	return err
}

// errorWithStderr augments err with the trimmed stderr of a failed command so callers
// surface why a subprocess failed instead of only its exit status.
func errorWithStderr(err error, stderr string) error {
	if trimmed := strings.TrimSpace(stderr); trimmed != "" {
		return errors.Errorf("%s: %s", err, trimmed)
	}
	return err
}

// errorWithCommandStderr augments the error of a command run with (*exec.Cmd).Output with
// the stderr that Output captured, so a failing subprocess reports why it failed rather
// than only "exit status 1".
func errorWithCommandStderr(err error) error {
	var exitErr *exec.ExitError
	if errors.As(err, &exitErr) {
		return errorWithStderr(err, string(exitErr.Stderr))
	}
	return err
}

func generatePythonStubs(module modulegen.ModuleInputs) error {
	venvName := ".venv"
	pythonCmd := findPythonCommand()
	if pythonCmd == "" {
		return errors.New("cannot generate python stubs -- python runtime not found")
	}
	if err := createPythonVenv(pythonCmd, venvName); err != nil {
		return errors.Wrap(err, "cannot generate python stubs -- unable to create python virtual environment")
	}
	defer utils.UncheckedErrorFunc(func() error { return os.RemoveAll(venvName) })

	script, err := scripts.ReadFile(path.Join(scriptsPath, "generate_stubs.py"))
	if err != nil {
		return errors.Wrap(err, "cannot generate python stubs -- unable to open generator script")
	}

	pythonVenvPath := filepath.Join(venvName, "bin", "python3")
	if runtime.GOOS == osWindows {
		pythonVenvPath = filepath.Join(venvName, "Scripts", "python.exe")
	}
	//nolint: gosec,noctx
	cmd := exec.Command(pythonVenvPath, "-c", string(script), module.ResourceType,
		module.ResourceSubtype, module.Namespace, module.ModuleName, module.ModelName)
	out, err := cmd.Output()
	if err != nil {
		return errors.Wrap(errorWithCommandStderr(err), "cannot generate python stubs -- generator script encountered an error")
	}

	resourcePath := filepath.Join(module.ModuleName, "src", "models", fmt.Sprintf("%s.py", module.ModelSnake))
	//nolint:gosec
	resourceFile, err := os.Create(resourcePath)
	if err != nil {
		return errors.Wrap(err, "cannot generate python stubs -- unable to open file")
	}
	defer utils.UncheckedErrorFunc(resourceFile.Close)
	_, err = resourceFile.Write(out)
	if err != nil {
		return errors.Wrap(err, "cannot generate python stubs -- unable to write to file")
	}

	return nil
}

func getLatestSDKTag(ctx context.Context, cmd *cli.Command, language string, globalArgs globalArgs) (string, error) {
	var repo string
	switch language {
	case python:
		repo = "viam-python-sdk"
	case golang:
		repo = "rdk"
	case cpp:
		repo = "viam-cpp-sdk"
	default:
		return "", errors.New("cannot produce template -- unexpected language was selected")
	}
	debugf(cmd.Root().Writer, globalArgs.Debug, "Getting the latest release tag for %s", repo)
	url := fmt.Sprintf("https://api.github.com/repos/viamrobotics/%s/releases", repo)

	req, err := http.NewRequestWithContext(ctx, http.MethodGet, url, nil)
	if err != nil {
		return "", errors.Wrapf(err, "cannot get latest %s release", repo)
	}
	//nolint:bodyclose
	resp, err := http.DefaultClient.Do(req)
	if err != nil {
		return "", errors.Wrapf(err, "cannot get latest %s release", repo)
	}
	defer utils.UncheckedErrorFunc(resp.Body.Close)
	if resp.StatusCode != http.StatusOK {
		return "", errors.Errorf("unexpected http GET status: %s", resp.Status)
	}
	var result interface{}
	err = json.NewDecoder(resp.Body).Decode(&result)
	if err != nil {
		return "", errors.Wrap(err, "could not decode json")
	}
	releases := result.([]interface{})
	if len(releases) == 0 {
		return "", errors.Errorf("could not get latest %s release", repo)
	}
	latest := releases[0]
	version := latest.(map[string]interface{})["tag_name"].(string)
	debugf(cmd.Root().Writer, globalArgs.Debug, "\tLatest release for %s: %s", repo, version)
	return version, nil
}

func createModuleAndManifest(
	ctx context.Context, cmd *cli.Command, c *viamClient, module modulegen.ModuleInputs, globalArgs globalArgs,
) (string, error) {
	var moduleID moduleID
	var registryURL string
	if module.RegisterOnApp {
		debugf(cmd.Root().Writer, globalArgs.Debug, "Registering module with Viam")
		moduleResponse, err := c.createModule(ctx, module.ModuleName, module.OrgID)
		if err != nil {
			return "", errors.Wrap(err, "failed to register module")
		}
		moduleID, err = parseModuleID(moduleResponse.GetModuleId())
		if err != nil {
			return "", errors.Wrap(err, "failed to parse module identifier")
		}
		registryURL = moduleResponse.GetUrl()
	} else {
		debugf(cmd.Root().Writer, globalArgs.Debug, "Creating a local-only module")
		moduleID.name = module.ModuleName
		moduleID.prefix = module.Namespace
	}
	err := renderManifest(cmd, moduleID.String(), module, globalArgs, nil)
	if err != nil {
		return "", errors.Wrap(err, "failed to render manifest")
	}
	return registryURL, nil
}

// Create the README.md file.
func renderReadme(module modulegen.ModuleInputs) error {
	readmeTemplatePath, err := templates.Open(path.Join(templatesPath, defaultReadmeFilename))
	readmeDest := filepath.Join(module.ModuleName, defaultReadmeFilename)
	if err != nil {
		return err
	}
	defer utils.UncheckedErrorFunc(readmeTemplatePath.Close)
	tBytes, err := io.ReadAll(readmeTemplatePath)
	if err != nil {
		return err
	}

	tmpl, err := template.New(defaultReadmeFilename).Parse(string(tBytes))
	if err != nil {
		return err
	}

	//nolint:gosec
	destFile, err := os.Create(readmeDest)
	if err != nil {
		return err
	}
	defer utils.UncheckedErrorFunc(destFile.Close)

	err = tmpl.Execute(destFile, module)
	if err != nil {
		return err
	}
	return nil
}

// Create the model documentation file.
func renderModelDoc(module modulegen.ModuleInputs) error {
	const modelDocTemplate = "MODEL_DOC.md"
	modelDocTemplatePath, err := templates.Open(path.Join(templatesPath, modelDocTemplate))
	if err != nil {
		return err
	}
	defer utils.UncheckedErrorFunc(modelDocTemplatePath.Close)

	tBytes, err := io.ReadAll(modelDocTemplatePath)
	if err != nil {
		return err
	}

	tmpl, err := template.New(modelDocTemplate).Parse(string(tBytes))
	if err != nil {
		return err
	}

	modelDocDest := filepath.Join(module.ModuleName, module.ModelReadmeLink)
	//nolint:gosec
	destFile, err := os.Create(modelDocDest)
	if err != nil {
		return err
	}
	defer utils.UncheckedErrorFunc(destFile.Close)

	err = tmpl.Execute(destFile, module)
	if err != nil {
		return err
	}
	return nil
}

// Create the meta.json manifest. If appInfo is non-nil, app-specific fields are added.
func renderManifest(
	cmd *cli.Command, moduleID string, module modulegen.ModuleInputs, globalArgs globalArgs, appInfo *appInputs,
) error {
	debugf(cmd.Root().Writer, globalArgs.Debug, "Rendering module manifest")

	visibility := module.Visibility
	if !slices.Contains(visibilityOption, visibility) {
		visibility = moduleVisibilityPrivate
		warningf(cmd.Root().Writer, "Defaulting to private due to invalid visibility '%q' - You can change this later", visibility)
	}

	manifest := ModuleManifest{
		Schema:       "https://dl.viam.dev/module.schema.json",
		ModuleID:     moduleID,
		Visibility:   visibility,
		Description:  fmt.Sprintf("Modular %s %s: %s", module.ResourceSubtype, module.ResourceType, module.ModelName),
		MarkdownLink: &module.ModuleReadmeLink,
	}

	if appInfo != nil {
		manifest.Description = fmt.Sprintf("%s app", appInfo.AppName)
		manifest.MarkdownLink = nil
		manifest.Models = []ModuleComponent{
			{
				API:   "rdk:component:generic",
				Model: fmt.Sprintf("%s:%s:webapp", module.Namespace, module.ModuleName),
			},
		}
		manifest.Apps = []AppComponent{
			{
				Name:       appInfo.AppName,
				Type:       appInfo.AppType,
				Entrypoint: "dist/index.html",
			},
		}
	}

	switch module.Language {
	case python:
		pythonEntrypoint := "dist/main"
		if runtime.GOOS == osWindows {
			pythonEntrypoint += ".exe"
			manifest.Build = &manifestBuildInfo{
				Setup: "setup.bat",
				Build: "build.bat",
				Path:  "dist/archive.tar.gz",
				Arch:  []string{"windows/amd64"},
			}
		} else {
			manifest.Build = &manifestBuildInfo{
				Setup: "./setup.sh",
				Build: "./build.sh",
				Path:  "dist/archive.tar.gz",
				Arch:  []string{"linux/amd64", "linux/arm64", "darwin/arm64"},
			}
		}
		manifest.Entrypoint = pythonEntrypoint
	case golang:
		moduleBinary := module.ModuleName
		if runtime.GOOS == osWindows {
			moduleBinary += ".exe"
			manifest.Build = &manifestBuildInfo{
				Setup: "go mod tidy",
				Build: "go build -tags no_cgo -o bin/" + moduleBinary +
					" cmd/module/main.go && tar czf module.tar.gz meta.json bin/" +
					moduleBinary,
				Path: "module.tar.gz",
				Arch: []string{"windows/amd64"},
			}
		} else {
			manifest.Build = &manifestBuildInfo{
				Setup: "make setup",
				Build: "make module.tar.gz",
				Path:  "module.tar.gz",
				Arch:  []string{"linux/amd64", "linux/arm64", "darwin/arm64"},
			}
		}
		manifest.Entrypoint = fmt.Sprintf("bin/%s", moduleBinary)
	case cpp:
		manifest.Build = &manifestBuildInfo{
			Build: "conan build . --build missing -s:a compiler.cppstd=17 --lockfile-partial",
			Path:  "build/Release/module.tar.gz",
			Arch:  []string{"linux/amd64", "linux/arm64"},
		}
		manifest.Entrypoint = fmt.Sprintf("bin/%s", module.ModuleName)
	}

	if err := writeManifest(filepath.Join(module.ModuleName, defaultManifestFilename), manifest); err != nil {
		return err
	}

	return nil
}

// readViamGenInfo reads and parses the .viam-gen-info file from the given directory.
func readViamGenInfo(dir string) (*modulegen.ModuleInputs, error) {
	infoPath := filepath.Join(dir, ".viam-gen-info")
	//nolint:gosec
	data, err := os.ReadFile(infoPath)
	if err != nil {
		if errors.Is(err, os.ErrNotExist) {
			return nil, errors.New(".viam-gen-info not found; run this command from within a module directory created with `viam module generate`")
		}
		return nil, errors.Wrap(err, "cannot read .viam-gen-info")
	}
	var info modulegen.ModuleInputs
	if err := json.Unmarshal(data, &info); err != nil {
		return nil, errors.Wrap(err, "cannot parse .viam-gen-info")
	}
	return &info, nil
}
