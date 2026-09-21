package cli

import (
	"context"
	"fmt"
	"io"
	"os"
	"os/exec"
	"path"
	"path/filepath"
	"runtime"
	"strings"
	"text/template"

	"charm.land/huh/v2"
	"charm.land/huh/v2/spinner"
	"github.com/pkg/errors"
	"github.com/urfave/cli/v3"
	"go.viam.com/utils"

	"go.viam.com/rdk/cli/module_generate/modulegen"
	gen "go.viam.com/rdk/cli/module_generate/scripts"
)

type addModelArgs struct {
	ResourceSubtype string
	ModelName       string
	DryRun          bool
}

type addAppArgs struct {
	AppName string
	AppType string
	DryRun  bool
}

// promptAddModelInputs asks for the resource and model name of a model being added to an
// existing module.
func promptAddModelInputs(ctx context.Context, cmd *cli.Command, module *modulegen.ModuleInputs) error {
	form, commit := addModelForm(module)
	if err := form.run(ctx, cmd); err != nil {
		return err
	}
	return commit()
}

// addModelForm builds the add-model form and the step that folds its answers back into module.
// The two are returned separately so tests can drive the form without a terminal.
func addModelForm(module *modulegen.ModuleInputs) (*formModel, func() error) {
	resource := canonicalResource(module.ResourceSubtype)
	form := newFormModel("viam module add-model", func(m *formModel) []huh.Field {
		options := buildResourceOptions()
		return []huh.Field{
			huh.NewNote().
				Title(fmt.Sprintf("Add a model to %s", module.ModuleName)).
				Description("This adds another resource model to a module you already have."),
			huh.NewSelect[string]().
				Title("Resource").
				Description("The component or service API your model implements.\n" +
					"Scroll with the arrow keys, and filter with `/`.").
				Options(options...).
				Height(min(len(options), 10)).
				Value(&resource),
			huh.NewInput().
				Title("Model name").
				Description("The name of the resource model to add.").
				PlaceholderFunc(func() string { return defaultModelName(resource) }, &resource).
				SuggestionsFunc(func() []string { return []string{defaultModelName(resource)} }, &resource).
				Value(&module.ModelName).
				Validate(m.whenAdvancing(func(s string) error { return validateName("model", s) })),
		}
	})
	return form, func() error {
		module.ResourceSubtype = flagSubtype(resource)
		return module.CheckResourceAndSetType()
	}
}

// promptAddAppInputs asks for the name and type of an app being added to an existing module.
func promptAddAppInputs(ctx context.Context, cmd *cli.Command, app *appInputs) error {
	return addAppForm(app).run(ctx, cmd)
}

// addAppForm builds the add-app form, separately from running it so tests can drive it.
func addAppForm(app *appInputs) *formModel {
	return newFormModel("viam module add-app", func(m *formModel) []huh.Field {
		return []huh.Field{
			huh.NewNote().
				Title("Add an app to this module").
				Description("This adds a web application to a module you already have.\n" +
					"For more detail see https://docs.viam.com/build-apps/hosting/overview/"),
			huh.NewInput().
				Title("App name").
				Description("The name of the web app entry in your module's meta.json.").
				Placeholder("my-app").
				Value(&app.AppName).
				Validate(m.whenAdvancing(func(s string) error { return validateName("app", s) })),
			huh.NewSelect[string]().
				Title("App type").
				Description("Single machine apps connect to one machine.\n"+
					"Multi machine apps can connect to several machines in your fleet.").
				Options(
					huh.NewOption("Single machine", appTypeSingleMachine),
					huh.NewOption("Multi machine", appTypeMultiMachine),
				).
				Value(&app.AppType),
		}
	})
}

// addGolangModelFile generates a Go source file with method stubs for the new model.
// It writes to <model_snake>.go in dir (the module root).
func addGolangModelFile(dir string, module modulegen.ModuleInputs) error {
	out, err := gen.RenderGoTemplates(module, true)
	if err != nil {
		return errors.Wrap(err, "generator script encountered an error")
	}
	modelFilePath := filepath.Join(dir, module.ModelSnake+".go")
	//nolint:gosec
	modelFile, err := os.Create(modelFilePath)
	if err != nil {
		return errors.Wrap(err, "unable to create model file")
	}
	defer utils.UncheckedErrorFunc(modelFile.Close)
	if _, err = modelFile.Write(out); err != nil {
		return errors.Wrap(err, "unable to write model file")
	}
	if err = runGoImports(modelFile); err != nil {
		return errors.Wrap(err, "unable to sort imports")
	}
	return nil
}

// addGoModelToMain updates cmd/module/main.go to register the new model in module.ModularMain.
// It inserts a resource.APIModel entry for the new model and adds the subtype import if needed.
func addGoModelToMain(mainGoPath string, module modulegen.ModuleInputs) error {
	//nolint:gosec
	data, err := os.ReadFile(mainGoPath)
	if err != nil {
		return errors.Wrap(err, "unable to read main.go")
	}
	content := string(data)

	// Add the import for the new resource subtype if it isn't already present.
	importPath := fmt.Sprintf("go.viam.com/rdk/%ss/%s", module.ResourceType, module.ResourceSubtype)
	if !strings.Contains(content, importPath) {
		newImport := fmt.Sprintf("\t%s \"go.viam.com/rdk/%ss/%s\"",
			module.ResourceSubtypeAlias, module.ResourceType, module.ResourceSubtype)
		// Insert before the blank line + closing ) that ends the import block.
		const importBlockEnd = "\n)\n\nfunc main"
		if !strings.Contains(content, importBlockEnd) {
			return errors.New("cannot locate import block closing in main.go; file may have been manually edited")
		}
		content = strings.Replace(content, importBlockEnd, "\n"+newImport+importBlockEnd, 1)
	}

	// Insert the new APIModel entry into the module.ModularMain(...) call.
	const mmFunc = "module.ModularMain("
	mmIdx := strings.Index(content, mmFunc)
	if mmIdx == -1 {
		return errors.New("cannot find module.ModularMain call in main.go; file may have been manually edited")
	}
	afterMM := content[mmIdx+len(mmFunc):]
	closingIdx := strings.Index(afterMM, ")")
	if closingIdx == -1 {
		return errors.New("cannot find closing ) of module.ModularMain in main.go")
	}
	newAPIModel := fmt.Sprintf(", resource.APIModel{%s.API, %s.%s}",
		module.ResourceSubtypeAlias, module.ModuleLowercase, module.ModelPascal)
	insertAt := mmIdx + len(mmFunc) + closingIdx
	content = content[:insertAt] + newAPIModel + content[insertAt:]

	//nolint:gosec
	return os.WriteFile(mainGoPath, []byte(content), 0o644)
}

// addPythonModelFiles generates a Python stub file for the new model in src/models/ and
// updates src/main.py to import it. Paths are relative to the current directory (the module root).
func addPythonModelFiles(module modulegen.ModuleInputs) error {
	venvName := ".venv"
	pythonCmd := findPythonCommand()
	if pythonCmd == "" {
		return errors.New("python runtime not found")
	}
	if err := createPythonVenv(pythonCmd, venvName); err != nil {
		return errors.Wrap(err, "unable to create python virtual environment")
	}
	defer utils.UncheckedErrorFunc(func() error { return os.RemoveAll(venvName) })

	script, err := scripts.ReadFile(path.Join(scriptsPath, "generate_stubs.py"))
	if err != nil {
		return errors.Wrap(err, "unable to open generator script")
	}

	pythonVenvPath := filepath.Join(venvName, "bin", "python3")
	if runtime.GOOS == osWindows {
		pythonVenvPath = filepath.Join(venvName, "Scripts", "python.exe")
	}
	//nolint: gosec,noctx
	stubCmd := exec.Command(pythonVenvPath, "-c", string(script), module.ResourceType,
		module.ResourceSubtype, module.Namespace, module.ModuleName, module.ModelName)
	out, err := stubCmd.Output()
	if err != nil {
		return errors.Wrap(errorWithCommandStderr(err), "generator script encountered an error")
	}

	resourcePath := filepath.Join("src", "models", fmt.Sprintf("%s.py", module.ModelSnake))
	//nolint:gosec
	resourceFile, err := os.Create(resourcePath)
	if err != nil {
		return errors.Wrap(err, "unable to create model file")
	}
	defer utils.UncheckedErrorFunc(resourceFile.Close)
	if _, err = resourceFile.Write(out); err != nil {
		return errors.Wrap(err, "unable to write model file")
	}

	mainPyPath := filepath.Join("src", "main.py")
	if err := addPythonModelImport(mainPyPath, module.ModelSnake, module.ModelPascal); err != nil {
		return errors.Wrap(err, "unable to update main.py")
	}
	return nil
}

// addPythonModelImport inserts an import line for the new model into src/main.py,
// placing it just before the `if __name__` guard so existing imports are undisturbed.
func addPythonModelImport(mainPyPath, modelSnake, modelPascal string) error {
	//nolint:gosec
	data, err := os.ReadFile(mainPyPath)
	if err != nil {
		return err
	}
	content := string(data)
	importLine := fmt.Sprintf("from models.%s import %s as %sModel\n", modelSnake, modelPascal, modelPascal)

	const mainGuard = "if __name__"
	if idx := strings.Index(content, mainGuard); idx != -1 {
		content = content[:idx] + importLine + "\n" + content[idx:]
	} else {
		content = strings.TrimRight(content, "\n") + "\n" + importLine + "\n"
	}
	//nolint:gosec
	return os.WriteFile(mainPyPath, []byte(content), 0o644)
}

// addCppModelFiles generates the per-model C++ source and header files in src/ and
// updates main.cpp and CMakeLists.txt to register and build the new model.
func addCppModelFiles(module modulegen.ModuleInputs) error {
	rendered, err := gen.RenderCppTemplates(module)
	if err != nil {
		return errors.Wrap(err, "generator script encountered an error")
	}

	srcDir := "src"
	if err := os.MkdirAll(srcDir, 0o750); err != nil {
		return errors.Wrap(err, "unable to create src directory")
	}

	filesToWrite := []struct {
		path string
		data []byte
	}{
		{filepath.Join(srcDir, fmt.Sprintf("%s.cpp", module.ModelSnake)), rendered.Type},
		{filepath.Join(srcDir, fmt.Sprintf("%s.hpp", module.ModelSnake)), rendered.Header},
	}
	for _, f := range filesToWrite {
		file, err := os.Create(f.path)
		if err != nil {
			return errors.Wrapf(err, "unable to create %s", f.path)
		}
		defer utils.UncheckedErrorFunc(file.Close)
		if _, err = file.Write(f.data); err != nil {
			return errors.Wrapf(err, "unable to write to %s", f.path)
		}
	}

	if err := addCppModelToMainCpp("main.cpp", module); err != nil {
		return errors.Wrap(err, "unable to update main.cpp")
	}
	if err := addCppModelToCMakeLists("CMakeLists.txt", module); err != nil {
		return errors.Wrap(err, "unable to update CMakeLists.txt")
	}
	return nil
}

// addCppModelToMainCpp inserts the include, model registration, and push_back for the new
// model into the existing main.cpp. It mirrors the structure emitted by main.cpp.in:
//
//	#include "<model>.hpp"                              ← after last local header include
//	viam::sdk::Model <model>_model(ns, module, model); ← before the mrs vector
//	auto <model>_mr = make_shared<ModelRegistration>(…);
//	mrs.push_back(<model>_mr);                         ← after mrs = {…};
func addCppModelToMainCpp(mainCppPath string, module modulegen.ModuleInputs) error {
	//nolint:gosec
	data, err := os.ReadFile(mainCppPath)
	if err != nil {
		return err
	}
	content := string(data)

	// 1. Add #include "<model>.hpp" after the last local (double-quoted) header include.
	newInclude := fmt.Sprintf("#include \"%s.hpp\"\n", module.ModelSnake)
	lastInclude := strings.LastIndex(content, "#include \"")
	if lastInclude == -1 {
		return errors.New("cannot find #include section in main.cpp")
	}
	endOfLine := strings.Index(content[lastInclude:], "\n")
	if endOfLine == -1 {
		return errors.New("malformed #include in main.cpp")
	}
	content = content[:lastInclude+endOfLine+1] + newInclude + content[lastInclude+endOfLine+1:]

	// 2. Insert model declaration + registration block just before the mrs vector.
	//    Variable names are prefixed with <model_snake> to avoid collisions across models.
	const mrsAnchor = "\n    std::vector<std::shared_ptr<viam::sdk::ModelRegistration>> mrs = {"
	mrsIdx := strings.Index(content, mrsAnchor)
	if mrsIdx == -1 {
		return errors.New("cannot find model registrations vector in main.cpp")
	}
	// %[1]s = ModelSnake  (underscores, for C++ variable names)
	// %[2]s = Namespace   (namespace string in model triple)
	// %[3]s = ModuleName  (original name with dashes, for model triple string)
	// %[4]s = ResourceSubtypePascal
	// %[5]s = ModelPascal (C++ class name)
	// %[6]s = ModuleSnake (underscores, for C++ namespace)
	// %[7]s = ModelName   (original name with dashes, for model triple string)
	registrationBlock := fmt.Sprintf(
		"\n    viam::sdk::Model %[1]s_model(\"%[2]s\", \"%[3]s\", \"%[7]s\");"+
			"\n\n    auto %[1]s_mr = std::make_shared<viam::sdk::ModelRegistration>("+
			"\n        viam::sdk::API::get<viam::sdk::%[4]s>(),"+
			"\n        %[1]s_model,"+
			"\n        [](viam::sdk::Dependencies deps, viam::sdk::ResourceConfig cfg) {"+
			"\n            return std::make_unique<%[6]s::%[5]s>(deps, cfg);"+
			"\n        },"+
			"\n        &%[6]s::%[5]s::validate);",
		module.ModelSnake, module.Namespace, module.ModuleName,
		module.ResourceSubtypePascal, module.ModelPascal, module.ModuleSnake, module.ModelName,
	)
	content = content[:mrsIdx] + registrationBlock + content[mrsIdx:]

	// 3. Append mrs.push_back(<model>_mr) right after the mrs = {…}; closing brace.
	//    Re-search after the insertion above shifted the index.
	mrsIdx = strings.Index(content, mrsAnchor)
	endOfMrs := strings.Index(content[mrsIdx:], "};\n")
	if endOfMrs == -1 {
		return errors.New("cannot find end of model registrations vector in main.cpp")
	}
	insertAt := mrsIdx + endOfMrs + len("};\n")
	content = content[:insertAt] +
		fmt.Sprintf("    mrs.push_back(%s_mr);\n", module.ModelSnake) +
		content[insertAt:]

	//nolint:gosec
	return os.WriteFile(mainCppPath, []byte(content), 0o644)
}

// addCppModelToCMakeLists adds the new model's source file to the add_executable target in
// CMakeLists.txt. The generated file always ends the add_executable block with:
//
//	    src/<model>.cpp
//	)
//
//	target_include_directories(…)
//
// so we locate that transition and insert before the closing paren.
func addCppModelToCMakeLists(cMakeListsPath string, module modulegen.ModuleInputs) error {
	//nolint:gosec
	data, err := os.ReadFile(cMakeListsPath)
	if err != nil {
		return err
	}
	content := string(data)

	const anchor = ")\n\ntarget_include_directories"
	idx := strings.Index(content, anchor)
	if idx == -1 {
		return errors.New("cannot find add_executable block in CMakeLists.txt")
	}
	content = content[:idx] + fmt.Sprintf("    src/%s.cpp\n", module.ModelSnake) + content[idx:]

	//nolint:gosec
	return os.WriteFile(cMakeListsPath, []byte(content), 0o644)
}

// renderModelDocToDir creates the per-model documentation file in dir.
func renderModelDocToDir(dir string, module modulegen.ModuleInputs) error {
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

	modelDocDest := filepath.Join(dir, module.ModelReadmeLink)
	//nolint:gosec
	destFile, err := os.Create(modelDocDest)
	if err != nil {
		return err
	}
	defer utils.UncheckedErrorFunc(destFile.Close)
	return tmpl.Execute(destFile, module)
}

// addModelToManifest appends a new model entry to the meta.json at manifestPath.
func addModelToManifest(manifestPath string, newModel modulegen.ModuleInputs) error {
	manifest, err := loadManifest(manifestPath)
	if err != nil {
		return err
	}
	markdownLink := newModel.ModelReadmeLink
	manifest.Models = append(manifest.Models, ModuleComponent{
		API:          newModel.API,
		Model:        newModel.ModelTriple,
		MarkdownLink: &markdownLink,
	})
	return writeManifest(manifestPath, manifest)
}

// AddModelAction adds a new model to an existing module created by `viam module generate`.
// Run this command from within the module directory.
func AddModelAction(ctx context.Context, cmd *cli.Command, args addModelArgs) error {
	// Read module-level info (language, namespace, name) from .viam-gen-info.
	genInfo, err := readViamGenInfo(".")
	if err != nil {
		return err
	}

	newModel := &modulegen.ModuleInputs{
		ModuleName:      genInfo.ModuleName,
		Language:        genInfo.Language,
		Namespace:       genInfo.Namespace,
		Visibility:      genInfo.Visibility,
		ResourceSubtype: args.ResourceSubtype,
		ModelName:       args.ModelName,
	}

	// If a resource subtype was provided via flag, validate it and set Resource/ResourceType.
	if err := newModel.CheckResourceAndSetType(); err != nil {
		return err
	}

	// Prompt for any fields that are still unset.
	if newModel.Resource == "" || newModel.ModelName == "" {
		if err := promptAddModelInputs(ctx, cmd, newModel); err != nil {
			return err
		}
	}

	populateAdditionalInfo(newModel)

	if args.DryRun {
		printf(cmd.Root().Writer, "Dry run: would add model %s to module %s", newModel.ModelTriple, newModel.ModuleName)
		return nil
	}

	gArgs, err := getGlobalArgs(cmd)
	if err != nil {
		return err
	}
	globalArgs := *gArgs

	sdkVersion, err := getLatestSDKTag(ctx, cmd, newModel.Language, globalArgs)
	if err != nil {
		return err
	}
	newModel.SDKVersion = sdkVersion[1:]
	// C++ SDK tags are of the form "release/vx.y.z"; strip the prefix so SDKVersion is always "x.y.z".
	if idx := strings.LastIndex(newModel.SDKVersion, "/"); idx != -1 {
		newModel.SDKVersion = strings.TrimPrefix(newModel.SDKVersion[idx+1:], "v")
	}

	// Guard against adding a model that already exists in meta.json.
	manifest, err := loadManifest(defaultManifestFilename)
	if err != nil {
		return errors.Wrap(err, "failed to read meta.json")
	}
	for _, model := range manifest.Models {
		if model.Model == newModel.ModelTriple {
			return fmt.Errorf("model %q already exists in meta.json", newModel.ModelTriple)
		}
	}

	s := spinner.New()
	var fatalError error
	action := func() {
		s.Title(fmt.Sprintf("Generating %s model stubs...", newModel.Language))
		switch newModel.Language {
		case golang:
			if err := addGolangModelFile(".", *newModel); err != nil {
				fatalError = errors.Wrap(err, "failed to generate Go model file")
				return
			}
			if err := addGoModelToMain(filepath.Join("cmd", "module", "main.go"), *newModel); err != nil {
				fatalError = errors.Wrap(err, "failed to update cmd/module/main.go")
				return
			}
		case python:
			if err := addPythonModelFiles(*newModel); err != nil {
				fatalError = errors.Wrap(err, "failed to generate Python model files")
				return
			}
		case cpp:
			if err := addCppModelFiles(*newModel); err != nil {
				fatalError = errors.Wrap(err, "failed to generate C++ model files")
				return
			}
		}

		s.Title("Generating model documentation...")
		if err := renderModelDocToDir(".", *newModel); err != nil {
			fatalError = errors.Wrap(err, "failed to generate model documentation")
			return
		}

		s.Title("Updating meta.json...")
		if err := addModelToManifest(defaultManifestFilename, *newModel); err != nil {
			fatalError = errors.Wrap(err, "failed to update meta.json")
			return
		}
	}

	if globalArgs.Debug {
		action()
	} else {
		s.Action(action)
		if err := s.Run(); err != nil {
			return err
		}
	}

	if fatalError != nil {
		return fatalError
	}

	cwd, err := os.Getwd()
	if err != nil {
		cwd = "."
	}
	printf(cmd.Root().Writer, "Model %s successfully added to module at %s", newModel.ModelTriple, cwd)
	return nil
}

// addGoWebappFile renders the app module.go template into webapp.go in dir.
// It uses appTemplateData (ModuleLowercase, Namespace, ModuleName) to fill the template.
func addGoWebappFile(dir string, data appTemplateData) error {
	const tmplName = "tmpl-module.go"
	appPath := path.Join(templatesPath, "app")
	tFile, err := templates.Open(path.Join(appPath, tmplName))
	if err != nil {
		return err
	}
	defer utils.UncheckedErrorFunc(tFile.Close)
	tBytes, err := io.ReadAll(tFile)
	if err != nil {
		return err
	}
	tmpl, err := template.New(tmplName).Parse(string(tBytes))
	if err != nil {
		return err
	}

	destPath := filepath.Join(dir, "webapp.go")
	//nolint:gosec
	destFile, err := os.Create(destPath)
	if err != nil {
		return errors.Wrap(err, "unable to create webapp.go")
	}
	defer utils.UncheckedErrorFunc(destFile.Close)
	if err := tmpl.Execute(destFile, data); err != nil {
		return errors.Wrap(err, "error rendering webapp template")
	}
	if err := runGoImports(destFile); err != nil {
		return errors.Wrap(err, "unable to sort imports")
	}
	return nil
}

// addGoWebappToMain updates cmd/module/main.go to register the webapp generic component model.
// It adds the generic import if absent and appends the webapp APIModel to ModularMain.
func addGoWebappToMain(mainGoPath string, data appTemplateData) error {
	//nolint:gosec
	rawData, err := os.ReadFile(mainGoPath)
	if err != nil {
		return errors.Wrap(err, "unable to read main.go")
	}
	content := string(rawData)

	// Add generic import if not already present.
	const genericPkg = "go.viam.com/rdk/components/generic"
	if !strings.Contains(content, genericPkg) {
		newImport := "\t\"go.viam.com/rdk/components/generic\""
		const importBlockEnd = "\n)\n\nfunc main"
		if !strings.Contains(content, importBlockEnd) {
			return errors.New("cannot locate import block in main.go; file may have been manually edited")
		}
		content = strings.Replace(content, importBlockEnd, "\n"+newImport+importBlockEnd, 1)
	}

	// Append the webapp APIModel entry to the module.ModularMain call.
	const mmFunc = "module.ModularMain("
	mmIdx := strings.Index(content, mmFunc)
	if mmIdx == -1 {
		return errors.New("cannot find module.ModularMain call in main.go; file may have been manually edited")
	}
	afterMM := content[mmIdx+len(mmFunc):]
	closingIdx := strings.Index(afterMM, ")")
	if closingIdx == -1 {
		return errors.New("cannot find closing ) of module.ModularMain in main.go")
	}
	newAPIModel := fmt.Sprintf(", resource.APIModel{API: generic.API, Model: %s.Model}", data.ModuleLowercase)
	insertAt := mmIdx + len(mmFunc) + closingIdx
	content = content[:insertAt] + newAPIModel + content[insertAt:]

	//nolint:gosec
	return os.WriteFile(mainGoPath, []byte(content), 0o644)
}

// addAppStaticFiles copies the static app files (auth.js, dist/index.html) into dir.
// Existing files are not overwritten so that user-created frontends are preserved.
func addAppStaticFiles(dir string) error {
	appTemplatePath := path.Join(templatesPath, "app")

	// Ensure the dist/ directory exists.
	distDir := filepath.Join(dir, "dist")
	if err := os.MkdirAll(distDir, 0o750); err != nil {
		return errors.Wrap(err, "unable to create dist directory")
	}

	// Copy static files only if they don't already exist.
	for _, f := range []struct{ src, dst string }{
		{"auth.js", filepath.Join(dir, "auth.js")},
		{path.Join("dist", "index.html"), filepath.Join(distDir, "index.html")},
	} {
		if _, err := os.Stat(f.dst); err == nil {
			// File already exists; leave it untouched.
			continue
		}
		srcFile, err := templates.Open(path.Join(appTemplatePath, f.src))
		if err != nil {
			return errors.Wrapf(err, "unable to open template %s", f.src)
		}
		defer utils.UncheckedErrorFunc(srcFile.Close)
		srcBytes, err := io.ReadAll(srcFile)
		if err != nil {
			return errors.Wrapf(err, "unable to read template %s", f.src)
		}
		//nolint:gosec
		if err := os.WriteFile(f.dst, srcBytes, 0o644); err != nil {
			return errors.Wrapf(err, "unable to write %s", f.dst)
		}
	}
	return nil
}

// updateMakefileForApp patches the Makefile of an existing Go module so that dist/ is included
// in the archive and vmodutils is fetched in the setup target. The function requires that
// MODULE_BINARY and module.tar.gz exist in the Makefile; it is otherwise tolerant of
// customizations to their values and prerequisites. The function is idempotent.
func updateMakefileForApp(makefilePath string) error {
	//nolint:gosec
	data, err := os.ReadFile(makefilePath)
	if err != nil {
		return errors.Wrap(err, "unable to read Makefile")
	}
	content := string(data)

	// Idempotency guard: if ENTRYPOINT is already declared (any value), nothing to do.
	const entrypointDecl = "ENTRYPOINT := dist/index.html"
	if strings.Contains(content, "ENTRYPOINT :=") {
		return nil
	}

	// 1. Declare ENTRYPOINT immediately after the MODULE_BINARY := line (any value).
	const moduleBinaryAssign = "MODULE_BINARY :="
	mbIdx := strings.Index(content, moduleBinaryAssign)
	if mbIdx == -1 {
		return errors.New("cannot locate MODULE_BINARY in Makefile")
	}
	eolIdx := strings.Index(content[mbIdx:], "\n")
	if eolIdx == -1 {
		return errors.New("malformed Makefile: MODULE_BINARY line has no newline")
	}
	content = content[:mbIdx+eolIdx+1] + entrypointDecl + "\n.DEFAULT_GOAL := all\n" + content[mbIdx+eolIdx+1:]

	// 2. Add dist to the TAR_FILES declaration line (any existing value).
	const tarFilesAssign = "TAR_FILES :="
	if tfIdx := strings.Index(content, tarFilesAssign); tfIdx != -1 {
		eol := strings.Index(content[tfIdx:], "\n")
		if eol != -1 {
			lineEnd := tfIdx + eol
			content = content[:lineEnd] + " dist" + content[lineEnd:]
		}
	}

	// 3. Add $(ENTRYPOINT) to the binary build target prerequisites (any existing deps).
	const binaryTarget = "$(MODULE_BINARY):"
	if bIdx := strings.Index(content, binaryTarget); bIdx != -1 {
		eol := strings.Index(content[bIdx:], "\n")
		if eol != -1 {
			lineEnd := bIdx + eol
			content = content[:lineEnd] + " $(ENTRYPOINT)" + content[lineEnd:]
		}
	}

	// 4. Add $(ENTRYPOINT) to module.tar.gz prerequisites (any existing deps).
	const tarTarget = "module.tar.gz:"
	if tIdx := strings.Index(content, tarTarget); tIdx != -1 {
		eol := strings.Index(content[tIdx:], "\n")
		if eol != -1 {
			lineEnd := tIdx + eol
			content = content[:lineEnd] + " $(ENTRYPOINT)" + content[lineEnd:]
		}
	} else {
		return errors.New("cannot locate module.tar.gz target in Makefile")
	}

	// 5. Add vmodutils to the setup target if not already present.
	const setupTarget = "\nsetup:\n"
	if strings.Contains(content, setupTarget) && !strings.Contains(content, "vmodutils") {
		content = strings.Replace(content, setupTarget,
			setupTarget+"\tgo get github.com/erh/vmodutils@latest\n", 1)
	}

	//nolint:gosec
	return os.WriteFile(makefilePath, []byte(content), 0o644)
}

// addAppToManifest appends a new application entry (and its webapp model if absent) to meta.json.
func addAppToManifest(manifestPath string, app *appInputs, data appTemplateData) error {
	manifest, err := loadManifest(manifestPath)
	if err != nil {
		return err
	}

	// Register the webapp generic component model if it isn't already listed.
	webappModel := fmt.Sprintf("%s:%s:webapp", data.Namespace, data.ModuleName)
	hasWebappModel := false
	for _, m := range manifest.Models {
		if m.Model == webappModel {
			hasWebappModel = true
			break
		}
	}
	if !hasWebappModel {
		manifest.Models = append(manifest.Models, ModuleComponent{
			API:   "rdk:component:generic",
			Model: webappModel,
		})
	}

	manifest.Apps = append(manifest.Apps, AppComponent{
		Name:       app.AppName,
		Type:       app.AppType,
		Entrypoint: "dist/index.html",
	})

	return writeManifest(manifestPath, manifest)
}

// AddAppAction adds a web application to an existing Go module created by `viam module generate`.
// Run this command from within the module directory.
func AddAppAction(ctx context.Context, cmd *cli.Command, args addAppArgs) error {
	// Read module-level info (language, namespace, name) from .viam-gen-info.
	genInfo, err := readViamGenInfo(".")
	if err != nil {
		return err
	}

	if genInfo.Language != golang {
		return fmt.Errorf("add-app currently only supports Go modules (this module uses %s); "+
			"build-system integration for other languages is not yet implemented", genInfo.Language)
	}

	app := &appInputs{
		AppName: args.AppName,
		AppType: args.AppType,
	}

	// Prompt for any fields that are still unset.
	if app.AppName == "" || app.AppType == "" {
		if err := promptAddAppInputs(ctx, cmd, app); err != nil {
			return err
		}
	}

	gArgs, err := getGlobalArgs(cmd)
	if err != nil {
		return err
	}
	globalArgs := *gArgs

	if args.DryRun {
		printf(cmd.Root().Writer, "Dry run: would add app %q to module %s", app.AppName, genInfo.ModuleName)
		return nil
	}

	// Guard against adding an app that already exists in meta.json.
	manifest, err := loadManifest(defaultManifestFilename)
	if err != nil {
		return errors.Wrap(err, "failed to read meta.json")
	}
	for _, existingApp := range manifest.Apps {
		if existingApp.Name == app.AppName {
			return fmt.Errorf("app %q already exists in meta.json", app.AppName)
		}
	}

	data := appTemplateData{
		ModuleName:      genInfo.ModuleName,
		ModuleLowercase: strings.ReplaceAll(strings.ToLower(genInfo.ModuleName), "-", ""),
		AppName:         app.AppName,
		AppType:         app.AppType,
		Namespace:       genInfo.Namespace,
		Visibility:      genInfo.Visibility,
		ConfigName:      "WebappConfig",
	}

	s := spinner.New()
	var fatalError error
	action := func() {
		s.Title("Generating webapp component file...")
		if err := addGoWebappFile(".", data); err != nil {
			fatalError = errors.Wrap(err, "failed to generate webapp.go")
			return
		}
		if err := addGoWebappToMain(filepath.Join("cmd", "module", "main.go"), data); err != nil {
			fatalError = errors.Wrap(err, "failed to update cmd/module/main.go")
			return
		}

		s.Title("Setting up app directories and static files...")
		if err := addAppStaticFiles("."); err != nil {
			fatalError = errors.Wrap(err, "failed to set up app files")
			return
		}
		if err := updateMakefileForApp("Makefile"); err != nil {
			fatalError = errors.Wrap(err, "failed to update Makefile")
			return
		}

		s.Title("Updating meta.json...")
		if err := addAppToManifest(defaultManifestFilename, app, data); err != nil {
			fatalError = errors.Wrap(err, "failed to update meta.json")
			return
		}
	}

	if globalArgs.Debug {
		action()
	} else {
		s.Action(action)
		if err := s.Run(); err != nil {
			return err
		}
	}

	if fatalError != nil {
		return fatalError
	}

	cwd, err := os.Getwd()
	if err != nil {
		cwd = "."
	}
	printf(cmd.Root().Writer, "App %q successfully added to module at %s", app.AppName, cwd)
	printAppBuildNextSteps(cmd.Root().Writer, "")
	return nil
}
