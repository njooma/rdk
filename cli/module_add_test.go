package cli

import (
	"testing"

	tea "charm.land/bubbletea/v2"
	"charm.land/huh/v2"
	"go.viam.com/test"

	"go.viam.com/rdk/cli/module_generate/modulegen"
)

func TestAddModelFormCollectsInputs(t *testing.T) {
	module := &modulegen.ModuleInputs{ModuleName: "my-module", Language: golang, Namespace: "my-org"}
	form, commit := addModelForm(module)
	d := newTeaDriver(t, form)

	// The resource picker opens on the first entry, so filter down to the one we want.
	d.typeText("/motor")
	d.keys(key(tea.KeyEnter)) // accept the filter
	d.keys(key(tea.KeyEnter)) // choose the highlighted resource
	d.typeText("my-motor")
	d.keys(key(tea.KeyEnter))

	test.That(t, form.form.State, test.ShouldEqual, huh.StateCompleted)
	test.That(t, commit(), test.ShouldBeNil)
	test.That(t, module.ResourceSubtype, test.ShouldEqual, "motor")
	test.That(t, module.ResourceType, test.ShouldEqual, "component")
	test.That(t, module.ModelName, test.ShouldEqual, "my-motor")
}

func TestAddModelFormPrepopulatesResourceFromFlag(t *testing.T) {
	module := &modulegen.ModuleInputs{ModuleName: "my-module", ResourceSubtype: "generic-service"}
	form, commit := addModelForm(module)
	d := newTeaDriver(t, form)

	d.keys(key(tea.KeyEnter)) // resource is already on the flag's value
	d.typeText("my-thing")
	d.keys(key(tea.KeyEnter))

	test.That(t, form.form.State, test.ShouldEqual, huh.StateCompleted)
	test.That(t, commit(), test.ShouldBeNil)
	// "generic" alone is ambiguous, so the round trip has to preserve the service half.
	test.That(t, module.ResourceSubtype, test.ShouldEqual, "generic")
	test.That(t, module.ResourceType, test.ShouldEqual, "service")
	test.That(t, module.ModelName, test.ShouldEqual, "my-thing")
}

func TestAddAppFormCollectsInputs(t *testing.T) {
	app := &appInputs{}
	form := addAppForm(app)
	d := newTeaDriver(t, form)

	d.typeText("my-app")
	d.keys(key(tea.KeyEnter))
	d.keys(key(tea.KeyDown), key(tea.KeyEnter)) // single -> multi machine

	test.That(t, form.form.State, test.ShouldEqual, huh.StateCompleted)
	test.That(t, app.AppName, test.ShouldEqual, "my-app")
	test.That(t, app.AppType, test.ShouldEqual, appTypeMultiMachine)
}

// TestAddFormsStillValidate guards that the add prompts enforce their rules going forward,
// the same as the wizard's pages do.
func TestAddFormsStillValidate(t *testing.T) {
	app := &appInputs{}
	form := addAppForm(app)
	d := newTeaDriver(t, form)

	d.keys(key(tea.KeyEnter)) // empty app name
	test.That(t, len(form.form.Errors()), test.ShouldEqual, 1)
	test.That(t, form.form.State, test.ShouldEqual, huh.StateNormal)

	d.typeText("my-app")
	d.keys(key(tea.KeyEnter))
	test.That(t, form.form.Errors(), test.ShouldBeEmpty)
}
