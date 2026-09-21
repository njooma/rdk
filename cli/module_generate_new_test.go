package cli

import (
	"errors"
	"os"
	"path/filepath"
	"reflect"
	"strings"
	"testing"
	"time"

	"charm.land/bubbles/v2/cursor"
	bubblespinner "charm.land/bubbles/v2/spinner"
	tea "charm.land/bubbletea/v2"
	"charm.land/huh/v2"
	"go.viam.com/test"

	"go.viam.com/rdk/cli/module_generate/modulegen"
)

// wizardDriver feeds messages to a generateWizard the way the bubbletea runtime would,
// draining the commands each update returns so that huh's own field and group transitions
// actually happen.
type teaDriver struct {
	t       *testing.T
	model   tea.Model
	pending []chan tea.Msg
}

// wizardDriver is a teaDriver that keeps the generate wizard handy for assertions.
type wizardDriver struct {
	*teaDriver
	wizard *generateWizard
}

// newWizardDriver builds a driver over a wizard that believes the user is already signed in,
// so tests never reach the OAuth device flow.
func newWizardDriver(t *testing.T, args *moduleArgs) *wizardDriver {
	t.Helper()
	conf := &Config{Auth: &token{User: userData{Email: "someone@viam.com"}}}
	return newWizardDriverWithAuth(t, args, &viamClient{conf: conf}, true)
}

func newWizardDriverWithAuth(t *testing.T, args *moduleArgs, client *viamClient, signedIn bool) *wizardDriver {
	t.Helper()
	wizard := newGenerateWizard(t.Context(), args, client, signedIn)
	return &wizardDriver{teaDriver: newTeaDriver(t, wizard), wizard: wizard}
}

func newTeaDriver(t *testing.T, model tea.Model) *teaDriver {
	t.Helper()
	d := &teaDriver{t: t, model: model}
	t.Cleanup(d.drain)
	d.send(model.Init())
	return d
}

// send feeds msgs to the wizard and settles every command they produce.
func (d *teaDriver) send(first tea.Cmd, msgs ...tea.Msg) {
	d.t.Helper()
	queue := make([]tea.Msg, 0, len(msgs)+1)
	if msg := d.run(first); msg != nil {
		queue = append(queue, msg)
	}
	queue = append(queue, msgs...)

	for steps := 0; len(queue) > 0; steps++ {
		test.That(d.t, steps < 2000, test.ShouldBeTrue)
		msg := queue[0]
		queue = queue[1:]

		// tea.Batch and tea.Sequence deliver a []tea.Cmd wrapped in a message; unwrap those
		// instead of handing them to the model, which is what the runtime does.
		if cmds, ok := unwrapCmds(msg); ok {
			for _, cmd := range cmds {
				if next := d.run(cmd); next != nil {
					queue = append(queue, next)
				}
			}
			continue
		}
		updated, cmd := d.model.Update(msg)
		d.model = updated
		if msg := d.run(cmd); msg != nil {
			queue = append(queue, msg)
		}
	}
}

func (d *teaDriver) keys(msgs ...tea.Msg) {
	d.t.Helper()
	d.send(nil, msgs...)
}

// typeText enters s a keypress at a time, the way a terminal delivers it.
func (d *teaDriver) typeText(s string) {
	d.t.Helper()
	for _, r := range s {
		d.keys(tea.KeyPressMsg{Code: r, Text: string(r)})
	}
}

// blinkSettle is comfortably longer than the cursor blink interval that makes the commands
// below outlive a single update.
const blinkSettle = 2 * time.Second

// run executes cmd, giving up on anything that does not come back promptly. huh's text inputs
// return a cursor-blink command that only resolves after the blink interval, and feeding
// those back would leave the driver ticking forever instead of draining.
func (d *teaDriver) run(cmd tea.Cmd) tea.Msg {
	if cmd == nil {
		return nil
	}
	done := make(chan tea.Msg, 1)
	go func() { done <- cmd() }()
	select {
	case msg := <-done:
		switch msg.(type) {
		case cursor.BlinkMsg, bubblespinner.TickMsg:
			return nil
		}
		return msg
	case <-time.After(20 * time.Millisecond):
		// Abandoned, but still running: remember it so drain can wait it out.
		d.pending = append(d.pending, done)
		return nil
	}
}

// drain waits for the abandoned commands to finish, so the package's goroutine leak check
// does not see the driver's leftovers.
func (d *teaDriver) drain() {
	deadline := time.After(blinkSettle)
	for _, done := range d.pending {
		select {
		case <-done:
		case <-deadline:
			d.t.Error("timed out waiting for abandoned tea commands to settle")
			return
		}
	}
}

func unwrapCmds(msg tea.Msg) ([]tea.Cmd, bool) {
	v := reflect.ValueOf(msg)
	if !v.IsValid() || v.Kind() != reflect.Slice || v.Type().Elem() != reflect.TypeOf(tea.Cmd(nil)) {
		return nil, false
	}
	cmds := make([]tea.Cmd, 0, v.Len())
	for i := range v.Len() {
		cmd, _ := v.Index(i).Interface().(tea.Cmd)
		cmds = append(cmds, cmd)
	}
	return cmds, true
}

func key(code rune) tea.Msg { return tea.KeyPressMsg{Code: code} }

func TestWizardCollectsModuleInputs(t *testing.T) {
	t.Chdir(t.TempDir())
	args := &moduleArgs{}
	d := newWizardDriver(t, args)
	w := d.wizard

	// Page 1: project type. "Module" is the first option.
	d.keys(key(tea.KeyEnter))
	// Page 2: name, language, visibility, namespace, register, dry run.
	d.typeText("my-module")
	d.keys(key(tea.KeyEnter))
	d.keys(key(tea.KeyDown), key(tea.KeyEnter)) // language: python -> go
	d.keys(key(tea.KeyEnter))                   // visibility: private
	d.typeText("my-org")
	d.keys(key(tea.KeyEnter))
	d.keys(key(tea.KeyEnter)) // register: no
	d.keys(key(tea.KeyEnter)) // dry run: no
	// Page 3: resource and model name. "arm component" is the first option.
	d.keys(key(tea.KeyEnter))
	d.typeText("my-arm")
	d.keys(key(tea.KeyEnter))

	test.That(t, w.form.State, test.ShouldEqual, huh.StateCompleted)
	test.That(t, args.Target, test.ShouldEqual, targetModule)
	test.That(t, args.Name, test.ShouldEqual, "my-module")
	test.That(t, args.Language, test.ShouldEqual, golang)
	test.That(t, args.Visibility, test.ShouldEqual, visibilityPrivate)
	test.That(t, args.Namespace, test.ShouldEqual, "my-org")
	test.That(t, args.ShouldRegister, test.ShouldBeFalse)
	test.That(t, args.DryRun, test.ShouldBeFalse)
	test.That(t, args.ModelArgs.ModelName, test.ShouldEqual, "my-arm")
	test.That(t, flagSubtype(w.resource), test.ShouldEqual, "arm")
}

func TestWizardSkipsPagesForAppOnly(t *testing.T) {
	t.Chdir(t.TempDir())
	args := &moduleArgs{}
	d := newWizardDriver(t, args)
	w := d.wizard

	// Page 1: move to "App".
	d.keys(key(tea.KeyDown), key(tea.KeyEnter))
	test.That(t, args.Target, test.ShouldEqual, targetApp)

	d.typeText("my-app")
	d.keys(key(tea.KeyEnter))
	// Language is constrained to Go for anything that ships an app.
	d.keys(key(tea.KeyEnter))
	test.That(t, args.Language, test.ShouldEqual, golang)
	d.keys(key(tea.KeyEnter)) // visibility
	d.typeText("my-org")
	d.keys(key(tea.KeyEnter))
	d.keys(key(tea.KeyEnter)) // register: no
	d.keys(key(tea.KeyEnter)) // dry run: no

	// The model page is hidden, so this lands directly on the app page.
	d.typeText("my-app")
	d.keys(key(tea.KeyEnter))
	d.keys(key(tea.KeyEnter)) // app type: single machine

	test.That(t, w.form.State, test.ShouldEqual, huh.StateCompleted)
	test.That(t, args.ModelArgs.ModelName, test.ShouldEqual, "")
	test.That(t, args.AppArgs.AppName, test.ShouldEqual, "my-app")
	test.That(t, args.AppArgs.AppType, test.ShouldEqual, appTypeSingleMachine)
}

func TestWizardPrepopulatesFromFlags(t *testing.T) {
	t.Chdir(t.TempDir())
	args := moduleArgsFromFlags(generateModuleArgs{
		GenerateType:    string(targetModule),
		Language:        cpp,
		Visibility:      moduleVisibilityPublic,
		PublicNamespace: "my-org",
		ResourceSubtype: "movement-sensor",
		ModelName:       "my-sensor",
	})
	test.That(t, args.normalize(), test.ShouldBeNil)
	d := newWizardDriver(t, &args)
	w := d.wizard

	// Everything that arrived as a flag is already selected, so only the name is left.
	test.That(t, w.resource, test.ShouldEqual, "movement_sensor component")

	d.keys(key(tea.KeyEnter)) // project type: module
	d.typeText("my-module")
	d.keys(key(tea.KeyEnter))                    // name
	d.keys(key(tea.KeyEnter), key(tea.KeyEnter)) // language, visibility
	d.keys(key(tea.KeyEnter), key(tea.KeyEnter)) // namespace, register
	d.keys(key(tea.KeyEnter))                    // dry run
	d.keys(key(tea.KeyEnter), key(tea.KeyEnter)) // resource, model name

	test.That(t, w.form.State, test.ShouldEqual, huh.StateCompleted)
	test.That(t, args.Language, test.ShouldEqual, cpp)
	test.That(t, args.Visibility, test.ShouldEqual, visibilityPublic)
	test.That(t, args.Namespace, test.ShouldEqual, "my-org")
	test.That(t, args.ModelArgs.ModelName, test.ShouldEqual, "my-sensor")
	test.That(t, flagSubtype(w.resource), test.ShouldEqual, "movement_sensor")
}

func TestWizardShowsSignInPageOnlyWhenNeeded(t *testing.T) {
	t.Chdir(t.TempDir())

	// Signed in already: no sign-in page, even when registering.
	args := &moduleArgs{}
	d := newWizardDriver(t, args)
	d.keys(key(tea.KeyEnter)) // module
	d.typeText("signed-in")
	d.keys(key(tea.KeyEnter))                    // name
	d.keys(key(tea.KeyEnter), key(tea.KeyEnter)) // language, visibility
	d.typeText("my-org")
	d.keys(key(tea.KeyEnter)) // namespace
	d.keys(key('y'))          // register: yes
	d.keys(key(tea.KeyEnter)) // dry run
	test.That(t, args.ShouldRegister, test.ShouldBeTrue)
	// Landing on the resource picker means the sign-in page was skipped.
	test.That(t, d.wizard.form.View(), test.ShouldContainSubstring, "Resource")

	// Signed out and registering: the sign-in page appears.
	signedOutArgs := &moduleArgs{}
	signedOut := newWizardDriverWithAuth(t, signedOutArgs, &viamClient{conf: &Config{}}, false)
	signedOut.keys(key(tea.KeyEnter))
	signedOut.typeText("signed-out")
	signedOut.keys(key(tea.KeyEnter))
	signedOut.keys(key(tea.KeyEnter), key(tea.KeyEnter))
	signedOut.typeText("my-org")
	signedOut.keys(key(tea.KeyEnter))
	signedOut.keys(key('y'))
	signedOut.keys(key(tea.KeyEnter)) // dry run
	test.That(t, signedOut.wizard.form.View(), test.ShouldContainSubstring, "Sign in")
	test.That(t, signedOut.wizard.auth.requested, test.ShouldBeFalse)
}

// TestWizardSignInPageKeepsURLIntact guards the note-markup escaping. huh renders "_" in a
// note description as italics, which would otherwise eat the underscore in a device URL.
func TestWizardSignInPageKeepsURLIntact(t *testing.T) {
	auth := &wizardAuth{
		phase:     authAwaitingBrowser,
		verifyURL: "https://viam-prod.fusionauth.io/device?user_code=ABCD-1234",
		userCode:  "ABCD-1234",
	}
	note := huh.NewNote().Description(auth.describe()).WithWidth(200)
	test.That(t, note.View(), test.ShouldContainSubstring, "?user_code=ABCD-1234")
}

func TestMissingFlagsDecidesWhetherToPrompt(t *testing.T) {
	complete := generateModuleArgs{
		GenerateType:    string(targetModule),
		Name:            "my-module",
		Language:        golang,
		Visibility:      moduleVisibilityPrivate,
		PublicNamespace: "my-org",
		ResourceSubtype: "arm",
		ModelName:       "my-arm",
	}
	args := moduleArgsFromFlags(complete)
	test.That(t, args.normalize(), test.ShouldBeNil)
	test.That(t, args.missingFlags(), test.ShouldBeEmpty)
	test.That(t, args.validate(), test.ShouldBeNil)

	// A module needs a resource; an app does not.
	noResource := complete
	noResource.ResourceSubtype = ""
	args = moduleArgsFromFlags(noResource)
	test.That(t, args.normalize(), test.ShouldBeNil)
	test.That(t, args.missingFlags(), test.ShouldResemble, []string{generalFlagResourceSubtype})

	appOnly := moduleArgsFromFlags(generateModuleArgs{
		GenerateType:    string(targetApp),
		Visibility:      moduleVisibilityPrivate,
		PublicNamespace: "my-org",
		AppName:         "my-app",
		AppType:         appTypeSingleMachine,
	})
	test.That(t, appOnly.normalize(), test.ShouldBeNil)
	// --name is inferred from --app-name, preserving the old app-only contract.
	test.That(t, appOnly.Name, test.ShouldEqual, "my-app")
	test.That(t, appOnly.missingFlags(), test.ShouldBeEmpty)
	test.That(t, appOnly.validate(), test.ShouldBeNil)

	both := moduleArgsFromFlags(generateModuleArgs{
		GenerateType:    string(targetModuleApp),
		Name:            "my-module",
		Visibility:      moduleVisibilityPrivate,
		PublicNamespace: "my-org",
		ResourceSubtype: "arm",
		ModelName:       "my-arm",
	})
	test.That(t, both.normalize(), test.ShouldBeNil)
	// Language is implied rather than prompted for, because app templates are Go-only.
	test.That(t, both.Language, test.ShouldEqual, golang)
	test.That(t, both.missingFlags(), test.ShouldResemble, []string{moduleFlagAppName, moduleFlagAppType})
}

func TestValidateRejectsBadFlagValues(t *testing.T) {
	base := moduleArgs{
		Target:     targetModule,
		Name:       "my-module",
		Language:   golang,
		Visibility: visibilityPrivate,
		Namespace:  "my-org",
		ModelArgs:  modelArgs{ResourceSubtype: "arm", ModelName: "my-arm"},
	}
	test.That(t, base.validate(), test.ShouldBeNil)

	for name, mutate := range map[string]func(a *moduleArgs){
		"bad target":     func(a *moduleArgs) { a.Target = "widget" },
		"bad name":       func(a *moduleArgs) { a.Name = "1-leading-digit" },
		"bad visibility": func(a *moduleArgs) { a.Visibility = "semi-public" },
		"no namespace":   func(a *moduleArgs) { a.Namespace = " " },
		"bad language":   func(a *moduleArgs) { a.Language = "rust" },
		"bad resource":   func(a *moduleArgs) { a.ModelArgs.ResourceSubtype = "flux-capacitor" },
		"bare generic":   func(a *moduleArgs) { a.ModelArgs.ResourceSubtype = "generic" },
		"bad model":      func(a *moduleArgs) { a.ModelArgs.ModelName = "" },
	} {
		args := base
		mutate(&args)
		test.That(t, args.validate(), test.ShouldNotBeNil)
		t.Logf("%s: %v", name, args.validate())
	}

	app := moduleArgs{
		Target: targetApp, Name: "my-app", Language: golang, Visibility: visibilityPrivate,
		Namespace: "my-org", AppArgs: appArgs{AppName: "my-app", AppType: "every_machine"},
	}
	test.That(t, app.validate(), test.ShouldNotBeNil)
}

// parseRerun turns a rendered rerun command back into the flag struct the CLI would parse,
// so tests can prove the printed command really does bypass the wizard.
func parseRerun(t *testing.T, command string) generateModuleArgs {
	t.Helper()
	test.That(t, strings.HasPrefix(command, "viam module generate "), test.ShouldBeTrue)

	var flags generateModuleArgs
	for _, token := range strings.Fields(strings.TrimPrefix(command, "viam module generate ")) {
		test.That(t, strings.HasPrefix(token, "--"), test.ShouldBeTrue)
		name, value, _ := strings.Cut(strings.TrimPrefix(token, "--"), "=")
		value = strings.Trim(value, "'")
		switch name {
		case moduleFlagGenerateType:
			flags.GenerateType = value
		case generalFlagName:
			flags.Name = value
		case moduleFlagLanguage:
			flags.Language = value
		case moduleFlagVisibility:
			flags.Visibility = value
		case moduleFlagPublicNamespace:
			flags.PublicNamespace = value
		case generalFlagResourceSubtype:
			flags.ResourceSubtype = value
		case generalFlagModelName:
			flags.ModelName = value
		case moduleFlagAppName:
			flags.AppName = value
		case moduleFlagAppType:
			flags.AppType = value
		case moduleFlagRegister:
			flags.Register = true
		default:
			t.Fatalf("rerun command used unknown flag --%s", name)
		}
	}
	return flags
}

func TestRerunCommandBypassesTheForm(t *testing.T) {
	for _, args := range []moduleArgs{
		{
			Target: targetModule, Name: "my-module", Language: python, Visibility: visibilityPrivate,
			Namespace: "my-org", ModelArgs: modelArgs{ResourceSubtype: "arm", ModelName: "my-arm"},
		},
		{
			Target: targetApp, Name: "my-app", Language: golang, Visibility: visibilityPublic,
			Namespace: "my-org", ShouldRegister: true,
			AppArgs: appArgs{AppName: "my-app", AppType: appTypeMultiMachine},
		},
		{
			Target: targetModuleApp, Name: "my-module", Language: golang, Visibility: visibilityPublicUnlisted,
			Namespace: "my-org", ModelArgs: modelArgs{ResourceSubtype: "generic-service", ModelName: "my-thing"},
			AppArgs: appArgs{AppName: "my-app", AppType: appTypeSingleMachine},
		},
	} {
		reparsed := moduleArgsFromFlags(parseRerun(t, args.rerunCommand()))
		test.That(t, reparsed.normalize(), test.ShouldBeNil)
		// No missing flags is exactly the condition under which the wizard is skipped.
		test.That(t, reparsed.missingFlags(), test.ShouldBeEmpty)
		test.That(t, reparsed.validate(), test.ShouldBeNil)
		test.That(t, reparsed, test.ShouldResemble, args)
	}
}

func TestRerunCommandQuotesUnsafeValues(t *testing.T) {
	args := moduleArgs{
		Target: targetModule, Name: "my-module", Language: golang, Visibility: visibilityPrivate,
		Namespace: "my org", ModelArgs: modelArgs{ResourceSubtype: "arm", ModelName: "my-arm"},
	}
	test.That(t, args.rerunCommand(), test.ShouldContainSubstring, "--public-namespace='my org'")
}

func TestResourceRoundTrip(t *testing.T) {
	// A flag value survives the trip into the picker and back out again.
	for flagValue, picker := range map[string]string{
		"arm":               "arm component",
		"movement-sensor":   "movement_sensor component",
		"generic-service":   "generic service",
		"generic-component": "generic component",
		"vision":            "vision service",
	} {
		test.That(t, canonicalResource(flagValue), test.ShouldEqual, picker)
		inputs := modulegen.ModuleInputs{ResourceSubtype: flagSubtype(picker)}
		test.That(t, inputs.CheckResourceAndSetType(), test.ShouldBeNil)
		test.That(t, inputs.Resource, test.ShouldEqual, picker)
	}
	// Every option the picker offers is a value the flag round trip accepts.
	for _, option := range buildResourceOptions() {
		inputs := modulegen.ModuleInputs{ResourceSubtype: flagSubtype(option.Value)}
		test.That(t, inputs.CheckResourceAndSetType(), test.ShouldBeNil)
		test.That(t, inputs.Resource, test.ShouldEqual, option.Value)
	}
	test.That(t, canonicalResource(""), test.ShouldEqual, modulegen.Resources[0])
	test.That(t, canonicalResource("not-a-resource"), test.ShouldEqual, modulegen.Resources[0])
	test.That(t, defaultModelName("arm component"), test.ShouldEqual, "my_arm")
	test.That(t, defaultModelName("generic component"), test.ShouldEqual, "my_generic_component")
}

func TestSDKVersionFromTag(t *testing.T) {
	test.That(t, sdkVersionFromTag("v0.44.0"), test.ShouldEqual, "0.44.0")
	test.That(t, sdkVersionFromTag("release/v0.20.1"), test.ShouldEqual, "0.20.1")
}

func planFor(t *testing.T, args moduleArgs) []string {
	t.Helper()
	inputs, err := args.toModuleInputs()
	test.That(t, err, test.ShouldBeNil)
	if args.Target.includesModule() {
		populateAdditionalInfo(inputs)
	}
	files, err := planGeneratedFiles(args, inputs)
	test.That(t, err, test.ShouldBeNil)
	paths := make([]string, 0, len(files))
	for _, f := range files {
		paths = append(paths, f.path)
	}
	return paths
}

func TestGenerationPlanListsTemplateOutputs(t *testing.T) {
	goModule := moduleArgs{
		Target: targetModule, Name: "my-module", Language: golang, Visibility: visibilityPrivate,
		Namespace: "my-org", ModelArgs: modelArgs{ResourceSubtype: "arm", ModelName: "my-arm"},
	}
	paths := planFor(t, goModule)
	for _, expected := range []string{
		"my-module/meta.json",
		"my-module/.viam-gen-info",
		"my-module/README.md",
		"my-module/my-org_my-module_my-arm.md",
		"my-module/.github/workflows/deploy.yml",
		"my-module/go.mod",
		"my-module/Makefile",
		"my-module/cmd/module/main.go",
		"my-module/module.go",
	} {
		test.That(t, paths, test.ShouldContain, expected)
	}
	// tmpl- prefixes never reach the generated project.
	for _, p := range paths {
		test.That(t, strings.Contains(p, templatePrefix), test.ShouldBeFalse)
	}

	pyModule := goModule
	pyModule.Language = python
	paths = planFor(t, pyModule)
	test.That(t, paths, test.ShouldContain, "my-module/requirements.txt")
	test.That(t, paths, test.ShouldContain, "my-module/src/main.py")
	test.That(t, paths, test.ShouldContain, "my-module/src/models/my_arm.py")

	cppModule := goModule
	cppModule.Language = cpp
	paths = planFor(t, cppModule)
	test.That(t, paths, test.ShouldContain, "my-module/main.cpp")
	test.That(t, paths, test.ShouldContain, "my-module/src/my_arm.hpp")
	test.That(t, paths, test.ShouldContain, "my-module/CMakeLists.txt")

	app := moduleArgs{
		Target: targetApp, Name: "my-app", Language: golang, Visibility: visibilityPrivate,
		Namespace: "my-org", AppArgs: appArgs{AppName: "my-app", AppType: appTypeSingleMachine},
	}
	paths = planFor(t, app)
	test.That(t, paths, test.ShouldContain, "my-app/meta.json")
	test.That(t, paths, test.ShouldContain, "my-app/auth.js")
	test.That(t, paths, test.ShouldContain, "my-app/dist/index.html")
	test.That(t, paths, test.ShouldContain, "my-app/module.go")
	// An app is not a generated module, so none of the module-only scaffolding appears.
	test.That(t, paths, test.ShouldNotContain, "my-app/.viam-gen-info")
	test.That(t, paths, test.ShouldNotContain, "my-app/.github/workflows/deploy.yml")
}

func TestGenerationPlanMarksUpdatedFiles(t *testing.T) {
	both := moduleArgs{
		Target: targetModuleApp, Name: "my-module", Language: golang, Visibility: visibilityPrivate,
		Namespace: "my-org", ModelArgs: modelArgs{ResourceSubtype: "arm", ModelName: "my-arm"},
		AppArgs: appArgs{AppName: "my-app", AppType: appTypeSingleMachine},
	}
	inputs, err := both.toModuleInputs()
	test.That(t, err, test.ShouldBeNil)
	populateAdditionalInfo(inputs)
	files, err := planGeneratedFiles(both, inputs)
	test.That(t, err, test.ShouldBeNil)

	actions := map[string]bool{}
	for _, f := range files {
		_, duplicated := actions[f.path]
		test.That(t, duplicated, test.ShouldBeFalse)
		actions[f.path] = f.updated
	}
	test.That(t, actions, test.ShouldContainKey, "my-module/webapp.go")
	test.That(t, actions["my-module/webapp.go"], test.ShouldBeFalse)
	// The Go templates create the Makefile in this same run, so it reads as created rather
	// than updated even though the app step edits it afterwards.
	test.That(t, actions["my-module/Makefile"], test.ShouldBeFalse)
	test.That(t, actions["my-module/meta.json"], test.ShouldBeFalse)
}

func TestPrintGenerationPlanWritesNothing(t *testing.T) {
	dir := t.TempDir()
	t.Chdir(dir)

	cmd := newTestContext(t, map[string]any{})
	args := moduleArgs{
		Target: targetModule, Name: "my-module", Language: golang, Visibility: visibilityPrivate,
		Namespace: "my-org", DryRun: true,
		ModelArgs: modelArgs{ResourceSubtype: "arm", ModelName: "my-arm"},
	}
	test.That(t, printGenerationPlan(cmd, args), test.ShouldBeNil)

	out, ok := cmd.Root().Writer.(*testWriter)
	test.That(t, ok, test.ShouldBeTrue)
	printed := strings.Join(out.messages, "")
	test.That(t, printed, test.ShouldContainSubstring, "Dry run: nothing was written.")
	test.That(t, printed, test.ShouldContainSubstring, "+ my-module/module.go")
	test.That(t, printed, test.ShouldContainSubstring, args.rerunCommand())

	entries, err := os.ReadDir(dir)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, entries, test.ShouldBeEmpty)
}

func TestRunGenerateStepsStopsAtFirstFatalError(t *testing.T) {
	cmd := newTestContext(t, map[string]any{"debug": true})
	var ran []string
	warnings, err := runGenerateSteps(cmd, true, []generateStep{
		{title: "first", run: func() error { ran = append(ran, "first"); return nil }},
		{title: "optional", optional: true, run: func() error {
			ran = append(ran, "optional")
			return errors.New("stubs failed")
		}},
		{title: "second", run: func() error { ran = append(ran, "second"); return errors.New("boom") }},
		{title: "never", run: func() error { ran = append(ran, "never"); return nil }},
	})
	test.That(t, ran, test.ShouldResemble, []string{"first", "optional", "second"})
	test.That(t, len(warnings), test.ShouldEqual, 1)
	test.That(t, err.Error(), test.ShouldEqual, "boom")
}

// TestGenerateAppEndToEnd exercises the whole app pipeline. App generation needs no network,
// unlike module generation, which resolves the latest SDK release before rendering stubs.
func TestGenerateAppEndToEnd(t *testing.T) {
	dir := t.TempDir()
	t.Chdir(dir)
	cmd := newTestContext(t, map[string]any{})

	args := moduleArgs{
		Target: targetApp, Name: "my-app", Language: golang, Visibility: visibilityPrivate,
		Namespace: "my-org", AppArgs: appArgs{AppName: "my-app", AppType: appTypeSingleMachine},
	}
	test.That(t, args.validate(), test.ShouldBeNil)
	// A local-only project needs no client at all.
	test.That(t, runGeneration(t.Context(), cmd, nil, args), test.ShouldBeNil)

	for _, f := range []string{"meta.json", "auth.js", "dist/index.html", "module.go", "Makefile", "go.mod"} {
		_, err := os.Stat(filepath.Join(dir, "my-app", f))
		test.That(t, err, test.ShouldBeNil)
	}
	page, err := os.ReadFile(filepath.Join(dir, "my-app", "dist", "index.html"))
	test.That(t, err, test.ShouldBeNil)
	test.That(t, string(page), test.ShouldContainSubstring, "<title>my-app</title>")

	manifest, err := loadManifest(filepath.Join(dir, "my-app", defaultManifestFilename))
	test.That(t, err, test.ShouldBeNil)
	test.That(t, manifest.ModuleID, test.ShouldEqual, "my-org:my-app")
	test.That(t, manifest.Visibility, test.ShouldEqual, moduleVisibilityPrivate)
	test.That(t, len(manifest.Apps), test.ShouldEqual, 1)
	test.That(t, manifest.Apps[0].Name, test.ShouldEqual, "my-app")
	test.That(t, manifest.Apps[0].Type, test.ShouldEqual, appTypeSingleMachine)

	// The dry run's plan describes the same files the real run produced.
	for _, planned := range planFor(t, args) {
		_, err := os.Stat(filepath.Join(dir, planned))
		test.That(t, err, test.ShouldBeNil)
	}
}

func shiftTab() tea.Msg { return tea.KeyPressMsg{Code: tea.KeyTab, Mod: tea.ModShift} }

// currentPage names the page on screen by a string unique to it.
func currentPage(d *wizardDriver) string {
	view := d.wizard.form.View()
	for _, page := range []struct{ marker, name string }{
		{"What would you like to build?", "type"},
		{"Namespace or organization ID", "details"},
		{"Sign in to Viam", "auth"},
		{"Model name", "model"},
		{"App type", "app"},
	} {
		if strings.Contains(view, page.marker) {
			return page.name
		}
	}
	return "unknown"
}

// TestWizardGoesBackFromAnUnfilledPage covers the trap huh sets by default: blurring a field
// runs its validator, and a group with errors refuses to hand back navigation. Arriving on a
// page and immediately retreating must work, precisely because nothing has been filled in.
func TestWizardGoesBackFromAnUnfilledPage(t *testing.T) {
	t.Chdir(t.TempDir())
	args := &moduleArgs{}
	d := newWizardDriver(t, args)

	d.keys(key(tea.KeyEnter)) // page 1 -> page 2, nothing typed
	test.That(t, currentPage(d), test.ShouldEqual, "details")

	d.keys(shiftTab())
	test.That(t, currentPage(d), test.ShouldEqual, "type")
	test.That(t, d.wizard.form.Errors(), test.ShouldBeEmpty)
}

// TestWizardGoesBackAfterAValidationError covers retreating from a page that is already
// showing an error, which is the other way the group-error gate locks a user in.
func TestWizardGoesBackAfterAValidationError(t *testing.T) {
	t.Chdir(t.TempDir())
	args := &moduleArgs{}
	d := newWizardDriver(t, args)
	d.keys(key(tea.KeyEnter))

	// Submitting the empty name raises its validator, as it should.
	d.keys(key(tea.KeyEnter))
	test.That(t, currentPage(d), test.ShouldEqual, "details")
	test.That(t, len(d.wizard.form.Errors()), test.ShouldEqual, 1)

	// Retreating clears it rather than being blocked by it.
	d.keys(shiftTab())
	test.That(t, currentPage(d), test.ShouldEqual, "type")
	test.That(t, d.wizard.form.Errors(), test.ShouldBeEmpty)
}

// TestWizardChangingTargetAfterGoingBackRehidesPages walks the reported scenario end to end:
// choose Module + App, go back, choose Module, and confirm the app page is gone.
func TestWizardChangingTargetAfterGoingBackRehidesPages(t *testing.T) {
	t.Chdir(t.TempDir())
	args := &moduleArgs{}
	d := newWizardDriver(t, args)

	d.keys(key(tea.KeyDown), key(tea.KeyDown), key(tea.KeyEnter))
	test.That(t, args.Target, test.ShouldEqual, targetModuleApp)

	d.keys(shiftTab())
	test.That(t, currentPage(d), test.ShouldEqual, "type")

	// Back on page 1, change the answer to Module and go forward again.
	d.keys(key(tea.KeyUp), key(tea.KeyUp), key(tea.KeyEnter))
	test.That(t, args.Target, test.ShouldEqual, targetModule)

	d.typeText("my-module")
	d.keys(key(tea.KeyEnter))
	// The language list widened back out to all three SDKs, and kept the Go that Module + App
	// had pinned it to, so moving up reaches Python again.
	t.Logf("language carried back from module+app: %q", args.Language)
	d.keys(key(tea.KeyUp), key(tea.KeyEnter))
	d.keys(key(tea.KeyEnter)) // visibility
	d.typeText("my-org")
	d.keys(key(tea.KeyEnter))
	d.keys(key(tea.KeyEnter)) // register
	d.keys(key(tea.KeyEnter)) // dry run
	test.That(t, currentPage(d), test.ShouldEqual, "model")

	d.keys(key(tea.KeyEnter))
	d.typeText("my-arm")
	d.keys(key(tea.KeyEnter))

	// The app page is hidden for a module-only project, so the form is done.
	test.That(t, d.wizard.form.State, test.ShouldEqual, huh.StateCompleted)
	test.That(t, args.Language, test.ShouldEqual, python)
	test.That(t, args.AppArgs.AppName, test.ShouldEqual, "")
}

// TestWizardStillBlocksInvalidInputGoingForward guards the other half of the contract: making
// the validators stand down on the way back must not let a bad value through on the way in.
func TestWizardStillBlocksInvalidInputGoingForward(t *testing.T) {
	t.Chdir(t.TempDir())
	args := &moduleArgs{}
	d := newWizardDriver(t, args)
	d.keys(key(tea.KeyEnter))

	d.keys(key(tea.KeyEnter)) // empty name
	test.That(t, currentPage(d), test.ShouldEqual, "details")
	test.That(t, len(d.wizard.form.Errors()), test.ShouldEqual, 1)

	d.typeText("1-starts-with-a-digit")
	d.keys(key(tea.KeyEnter))
	test.That(t, currentPage(d), test.ShouldEqual, "details")
	test.That(t, len(d.wizard.form.Errors()), test.ShouldEqual, 1)
	test.That(t, args.Name, test.ShouldNotEqual, "")
}

// TestWizardBacksThroughFieldsThenPages checks that the fix leaves huh's field-by-field walk
// intact: shift+tab steps back through a page's fields and only then changes page.
func TestWizardBacksThroughFieldsThenPages(t *testing.T) {
	t.Chdir(t.TempDir())
	args := &moduleArgs{}
	d := newWizardDriver(t, args)
	d.keys(key(tea.KeyEnter))
	d.typeText("my-module")
	d.keys(key(tea.KeyEnter))
	d.keys(key(tea.KeyEnter), key(tea.KeyEnter))
	d.typeText("my-org")
	d.keys(key(tea.KeyEnter))
	d.keys(key(tea.KeyEnter)) // register
	d.keys(key(tea.KeyEnter)) // dry run -> model page
	test.That(t, currentPage(d), test.ShouldEqual, "model")

	// Six fields on the details page, so the seventh press is the one that changes page.
	pages := make([]string, 0, 7)
	for range 7 {
		d.keys(shiftTab())
		pages = append(pages, currentPage(d))
	}
	test.That(t, pages, test.ShouldResemble,
		[]string{"details", "details", "details", "details", "details", "details", "type"})
	test.That(t, args.Name, test.ShouldEqual, "my-module")
	test.That(t, args.Namespace, test.ShouldEqual, "my-org")
}
