package cli

import (
	"bytes"
	"context"
	"fmt"
	"io/fs"
	"os"
	"path"
	"path/filepath"
	"regexp"
	"runtime"
	"slices"
	"strings"
	"sync"

	bubblespinner "charm.land/bubbles/v2/spinner"
	tea "charm.land/bubbletea/v2"
	"charm.land/huh/v2"
	"charm.land/huh/v2/spinner"
	"charm.land/lipgloss/v2"
	"github.com/pkg/errors"
	"github.com/russross/blackfriday/v2"
	"github.com/urfave/cli/v3"
	"go.viam.com/utils"

	"go.viam.com/rdk/cli/module_generate/modulegen"
)

type generateModuleArgs struct {
	GenerateType    string
	Name            string
	Language        string
	Visibility      string
	PublicNamespace string
	ResourceSubtype string
	ModelName       string
	Register        bool
	DryRun          bool
	AppName         string
	AppType         string
}

// moduleVisibility is the registry visibility of a generated module or app.
type moduleVisibility string

const (
	visibilityPrivate        moduleVisibility = moduleVisibilityPrivate
	visibilityPublicUnlisted moduleVisibility = moduleVisibilityPublicUnlisted
	visibilityPublic         moduleVisibility = moduleVisibilityPublic
)

// generateTarget is the kind of project the wizard was asked to scaffold.
type generateTarget string

const (
	targetModule    generateTarget = "module"
	targetApp       generateTarget = "app"
	targetModuleApp generateTarget = "module+app"
)

const (
	appTypeSingleMachine = "single_machine"
	appTypeMultiMachine  = "multi_machine"
)

var generateTargets = []generateTarget{targetModule, targetApp, targetModuleApp}

func (t generateTarget) includesModule() bool { return t == targetModule || t == targetModuleApp }

func (t generateTarget) includesApp() bool { return t == targetApp || t == targetModuleApp }

// modelArgs describes a single resource model provided by a module.
type modelArgs struct {
	ResourceSubtype string
	ModelName       string
}

// appArgs describes a single web app shipped by a module.
type appArgs struct {
	AppName string
	AppType string
}

// moduleArgs is the complete set of answers the generate wizard collects. Every field can
// also be supplied up front as a CLI flag; when all of the fields a target requires are
// already set, the wizard is skipped entirely.
type moduleArgs struct {
	Target         generateTarget
	Name           string
	Language       string
	Visibility     moduleVisibility
	Namespace      string
	ShouldRegister bool
	DryRun         bool
	ModelArgs      modelArgs
	AppArgs        appArgs
}

func moduleArgsFromFlags(flags generateModuleArgs) moduleArgs {
	return moduleArgs{
		Target:         generateTarget(flags.GenerateType),
		Name:           flags.Name,
		Language:       flags.Language,
		Visibility:     moduleVisibility(flags.Visibility),
		Namespace:      flags.PublicNamespace,
		ShouldRegister: flags.Register,
		DryRun:         flags.DryRun,
		ModelArgs:      modelArgs{ResourceSubtype: flags.ResourceSubtype, ModelName: flags.ModelName},
		AppArgs:        appArgs{AppName: flags.AppName, AppType: flags.AppType},
	}
}

// normalize fills in the values implied by other answers, so that both the wizard's
// pre-population and a wizard-less run see one consistent set of inputs.
func (a *moduleArgs) normalize() error {
	a.Target = generateTarget(strings.TrimSpace(string(a.Target)))
	if a.Target != "" && !slices.Contains(generateTargets, a.Target) {
		return errors.Errorf("invalid --%s %q: must be one of module, app, module+app", moduleFlagGenerateType, a.Target)
	}
	// `--generate-type app --app-name x` historically used the app name as the module name;
	// keep that working so --name stays optional for app-only runs.
	if a.Target == targetApp && a.Name == "" {
		a.Name = a.AppArgs.AppName
	}
	// The app templates are Go-only, so anything that ships an app is a Go module.
	if a.Target.includesApp() {
		a.Language = golang
	}
	return nil
}

// missingFlags lists the flags still needed to generate without showing the wizard.
func (a moduleArgs) missingFlags() []string {
	var missing []string
	if a.Target == "" {
		missing = append(missing, moduleFlagGenerateType)
	}
	if a.Name == "" {
		missing = append(missing, generalFlagName)
	}
	if a.Visibility == "" {
		missing = append(missing, moduleFlagVisibility)
	}
	if a.Namespace == "" {
		missing = append(missing, moduleFlagPublicNamespace)
	}
	if a.Target.includesModule() {
		if a.Language == "" {
			missing = append(missing, moduleFlagLanguage)
		}
		if a.ModelArgs.ResourceSubtype == "" {
			missing = append(missing, generalFlagResourceSubtype)
		}
		if a.ModelArgs.ModelName == "" {
			missing = append(missing, generalFlagModelName)
		}
	}
	if a.Target.includesApp() {
		if a.AppArgs.AppName == "" {
			missing = append(missing, moduleFlagAppName)
		}
		if a.AppArgs.AppType == "" {
			missing = append(missing, moduleFlagAppType)
		}
	}
	return missing
}

// validate applies the same rules the wizard enforces through its field validators, so a
// fully-flagged invocation that never shows the wizard fails just as early.
func (a moduleArgs) validate() error {
	if !slices.Contains(generateTargets, a.Target) {
		return errors.Errorf("invalid --%s %q: must be one of module, app, module+app", moduleFlagGenerateType, a.Target)
	}
	if err := validateName("project", a.Name); err != nil {
		return err
	}
	if !slices.Contains(visibilityOption, string(a.Visibility)) {
		return errors.Errorf("invalid --%s %q: must be one of %s",
			moduleFlagVisibility, a.Visibility, strings.Join(visibilityOption, ", "))
	}
	if strings.TrimSpace(a.Namespace) == "" {
		return errors.Errorf("--%s must not be empty", moduleFlagPublicNamespace)
	}
	if a.Target.includesModule() {
		if !slices.Contains(supportedModuleGenLanguages, a.Language) {
			return errors.Errorf("invalid --%s %q: must be one of %s",
				moduleFlagLanguage, a.Language, strings.Join(supportedModuleGenLanguages, ", "))
		}
		inputs := modulegen.ModuleInputs{ResourceSubtype: a.ModelArgs.ResourceSubtype}
		if err := inputs.CheckResourceAndSetType(); err != nil {
			return err
		}
		if err := validateName("model", a.ModelArgs.ModelName); err != nil {
			return err
		}
	}
	if a.Target.includesApp() {
		if err := validateName("app", a.AppArgs.AppName); err != nil {
			return err
		}
		if a.AppArgs.AppType != appTypeSingleMachine && a.AppArgs.AppType != appTypeMultiMachine {
			return errors.Errorf("invalid --%s %q: must be one of %s, %s",
				moduleFlagAppType, a.AppArgs.AppType, appTypeSingleMachine, appTypeMultiMachine)
		}
	}
	return nil
}

var projectNameRegexp = regexp.MustCompile(`^[a-zA-Z]+(?:[_\-a-zA-Z0-9]+)*$`)

// validateName enforces the naming rules shared by module, app, and model names.
func validateName(kind, name string) error {
	if name == "" {
		return errors.Errorf("%s name must not be empty", kind)
	}
	if !projectNameRegexp.MatchString(name) {
		return errors.Errorf("%s names can only contain alphanumeric characters, dashes, and\n"+
			"underscores, and must start with a letter", kind)
	}
	return nil
}

// toModuleInputs maps the collected answers onto the struct the template renderers consume.
func (a moduleArgs) toModuleInputs() (*modulegen.ModuleInputs, error) {
	inputs := &modulegen.ModuleInputs{
		ModuleName:      a.Name,
		Language:        a.Language,
		Visibility:      string(a.Visibility),
		Namespace:       a.Namespace,
		ResourceSubtype: a.ModelArgs.ResourceSubtype,
		ModelName:       a.ModelArgs.ModelName,
		RegisterOnApp:   a.ShouldRegister,
	}
	if a.Target.includesModule() {
		if err := inputs.CheckResourceAndSetType(); err != nil {
			return nil, err
		}
	}
	if inputs.Language == "" {
		inputs.Language = golang
	}
	return inputs, nil
}

var shellSafeRegexp = regexp.MustCompile(`^[A-Za-z0-9_@%+=:,./-]+$`)

func flagArg(name, value string) string {
	if !shellSafeRegexp.MatchString(value) {
		return fmt.Sprintf("--%s='%s'", name, strings.ReplaceAll(value, "'", `'\''`))
	}
	return fmt.Sprintf("--%s=%s", name, value)
}

// rerunCommand renders the `viam module generate` invocation that reproduces these answers
// without showing the wizard, so a dry run can be turned into a scripted real run.
func (a moduleArgs) rerunCommand() string {
	parts := []string{
		"viam module generate",
		flagArg(moduleFlagGenerateType, string(a.Target)),
		flagArg(generalFlagName, a.Name),
		flagArg(moduleFlagVisibility, string(a.Visibility)),
		flagArg(moduleFlagPublicNamespace, a.Namespace),
	}
	if a.Target.includesModule() {
		parts = append(parts,
			flagArg(moduleFlagLanguage, a.Language),
			flagArg(generalFlagResourceSubtype, a.ModelArgs.ResourceSubtype),
			flagArg(generalFlagModelName, a.ModelArgs.ModelName),
		)
	}
	if a.Target.includesApp() {
		parts = append(parts,
			flagArg(moduleFlagAppName, a.AppArgs.AppName),
			flagArg(moduleFlagAppType, a.AppArgs.AppType),
		)
	}
	if a.ShouldRegister {
		parts = append(parts, "--"+moduleFlagRegister)
	}
	return strings.Join(parts, " ")
}

// GenerateModuleActionV2 scaffolds a module, an app, or both, prompting for whatever was not
// supplied as a flag. Flags pre-populate the wizard; when every flag the chosen target needs
// is present the wizard is skipped and generation starts immediately.
func GenerateModuleActionV2(ctx context.Context, cmd *cli.Command, flags generateModuleArgs) error {
	args := moduleArgsFromFlags(flags)
	if err := args.normalize(); err != nil {
		return err
	}

	// Unlike newViamClient this does not fail when the user is signed out: the wizard offers
	// to sign them in, and a local-only project needs no credentials at all.
	client, err := newViamClientInner(ctx, cmd, false)
	if err != nil {
		return err
	}
	signedIn := client.ensureLoggedIn(ctx) == nil

	if missing := args.missingFlags(); len(missing) > 0 {
		if !isInteractive() {
			return errors.Errorf("missing required flags for non-interactive mode: --%s", strings.Join(missing, ", --"))
		}
		if signedIn, err = runGenerateWizard(ctx, cmd, &args, client, signedIn); err != nil {
			return err
		}
	}
	if err := args.validate(); err != nil {
		return err
	}
	if args.DryRun {
		return printGenerationPlan(cmd, args)
	}
	if args.ShouldRegister && !signedIn {
		return errors.Errorf("registering with Viam requires an account: run `viam login`, or drop --%s", moduleFlagRegister)
	}
	return runGeneration(ctx, cmd, client, args)
}

const wizardWidth = 88

// backKey is huh's own "back" binding, shared by every field type and advertised in the help
// footer. The wizard watches for it rather than binding a key of its own.
const backKey = "shift+tab"

// formNav tracks which way the user is moving through a form.
type formNav struct {
	retreating bool
}

// noteKey records the direction of travel before the form sees the key.
func (n *formNav) noteKey(msg tea.KeyPressMsg) { n.retreating = msg.String() == backKey }

// whenAdvancing wraps a field validator so that it only enforces its rule on the way forward.
// huh runs a field's validator whenever the field blurs, which includes blurring it to move
// back a page, and then refuses to leave a group that has errors. Validating on the way out
// would therefore trap the user on a page for the very reason they are trying to leave it:
// arriving on a page with an empty required field and immediately pressing shift+tab would
// raise "must not be empty" and silently swallow the navigation. Values are left untouched
// and get validated as normal the next time the user moves forward.
func (n *formNav) whenAdvancing(validate func(string) error) func(string) error {
	return func(s string) error {
		if n.retreating {
			return nil
		}
		return validate(s)
	}
}

// runTUI runs a model that owns the terminal, holding back anything the CLI's own helpers
// print in the meantime and flushing it once the program has given the terminal back.
func runTUI(ctx context.Context, cmd *cli.Command, model tea.Model) (tea.Model, error) {
	root := cmd.Root()
	stdout, stderr := root.Writer, root.ErrWriter
	held := &syncBuffer{}
	root.Writer, root.ErrWriter = held, held

	final, err := tea.NewProgram(model, tea.WithContext(ctx), tea.WithOutput(os.Stderr)).Run()

	root.Writer, root.ErrWriter = stdout, stderr
	if withheld := held.String(); withheld != "" {
		printf(stderr, "%s", strings.TrimRight(withheld, "\n"))
	}
	return final, err
}

// formModel hosts a single-page huh form, for the prompts that ask too little to be worth a
// wizard. It shares the wizard's chrome and its backwards-navigation behavior.
type formModel struct {
	formNav
	form     *huh.Form
	heading  string
	styles   wizardStyles
	finished bool
}

// newFormModel builds a one-page form. Fields come from a callback because they need the
// model's whenAdvancing wrapper, which cannot exist until the model does.
func newFormModel(heading string, fields func(*formModel) []huh.Field) *formModel {
	m := &formModel{heading: heading, styles: newWizardStyles(true)}
	m.form = huh.NewForm(huh.NewGroup(fields(m)...)).
		WithWidth(wizardWidth).
		WithTheme(huh.ThemeFunc(huh.ThemeCharm))
	return m
}

// Init implements tea.Model.
func (m *formModel) Init() tea.Cmd { return m.form.Init() }

// Update implements tea.Model.
func (m *formModel) Update(msg tea.Msg) (tea.Model, tea.Cmd) {
	switch msg := msg.(type) {
	case tea.KeyPressMsg:
		m.noteKey(msg)
	case tea.BackgroundColorMsg:
		m.styles = newWizardStyles(msg.IsDark())
	}

	var cmds []tea.Cmd
	form, cmd := m.form.Update(msg)
	if f, ok := form.(*huh.Form); ok {
		m.form = f
	}
	cmds = append(cmds, cmd)

	if m.form.State != huh.StateNormal && !m.finished {
		m.finished = true
		cmds = append(cmds, tea.Quit)
	}
	return m, tea.Batch(cmds...)
}

// View implements tea.Model.
func (m *formModel) View() tea.View {
	if m.finished {
		return tea.NewView("")
	}
	return tea.NewView(lipgloss.JoinVertical(lipgloss.Left,
		m.styles.header.Render(m.heading), "", m.form.View()))
}

// run shows the form and reports whether the user completed it.
func (m *formModel) run(ctx context.Context, cmd *cli.Command) error {
	if !isInteractive() {
		return errors.New("missing required flags for non-interactive mode")
	}
	final, err := runTUI(ctx, cmd, m)
	if err != nil {
		return err
	}
	if done, ok := final.(*formModel); !ok || done.form.State != huh.StateCompleted {
		return huh.ErrUserAborted
	}
	return nil
}

// wizardStyles are the lipgloss styles for the chrome the wizard draws around the huh form.
type wizardStyles struct {
	header lipgloss.Style
	hint   lipgloss.Style
	good   lipgloss.Style
	bad    lipgloss.Style
	link   lipgloss.Style
}

func newWizardStyles(isDark bool) wizardStyles {
	lightDark := lipgloss.LightDark(isDark)
	return wizardStyles{
		header: lipgloss.NewStyle().Bold(true).Padding(0, 1).
			Foreground(lipgloss.Color("#FFFFFF")).Background(lipgloss.Color("#7C3AED")),
		hint: lipgloss.NewStyle().Foreground(lightDark(lipgloss.Color("#6C6F85"), lipgloss.Color("#9399B2"))),
		good: lipgloss.NewStyle().Foreground(lightDark(lipgloss.Color("#3B7A57"), lipgloss.Color("#A6E3A1"))),
		bad:  lipgloss.NewStyle().Foreground(lightDark(lipgloss.Color("#B4436C"), lipgloss.Color("#F38BA8"))),
		link: lipgloss.NewStyle().Underline(true).
			Foreground(lightDark(lipgloss.Color("#1E66F5"), lipgloss.Color("#89B4FA"))),
	}
}

// authPhase tracks the sign-in the wizard can run without interrupting the form.
type authPhase int

const (
	authSignedOut authPhase = iota
	authRequestingCode
	authAwaitingBrowser
	authSignedIn
	authFailed
)

// wizardAuth drives the OAuth device flow from inside the wizard. The CLI's own login helper
// writes the verification URL straight to the terminal, which would corrupt the TUI, so this
// runs the same steps (discover, request a device code, poll for a token) as tea commands and
// renders the URL itself.
type wizardAuth struct {
	client *viamClient
	// ctx is held because the tea commands are built in Update, which has no context of its own.
	ctx context.Context //nolint:containedctx

	phase     authPhase
	email     string
	verifyURL string
	userCode  string
	err       error

	// requested is bound to the wizard's sign-in button. A huh field cannot run a tea command,
	// so Update watches this flag and starts the device flow when it flips.
	requested bool
}

func (a *wizardAuth) signedIn() bool { return a.phase == authSignedIn }

func (a *wizardAuth) pending() bool {
	return a.phase == authRequestingCode || a.phase == authAwaitingBrowser
}

func (a *wizardAuth) fail(err error) {
	a.phase = authFailed
	a.err = err
	// Reset the button so selecting it again retries.
	a.requested = false
}

type authCodeMsg struct {
	code      *deviceCodeResponse
	discovery *openIDDiscoveryResponse
	err       error
}

type authDoneMsg struct {
	email string
	err   error
}

// browserOpenedMsg gives the fire-and-forget browser launch a message to deliver.
type browserOpenedMsg struct{}

func (a *wizardAuth) requestCode() tea.Cmd {
	return func() tea.Msg {
		discovery, err := a.client.authFlow.loadOIDiscoveryEndpoint(a.ctx)
		if err != nil {
			return authCodeMsg{err: errors.Wrap(err, "failed retrieving discovery endpoint")}
		}
		code, err := a.client.authFlow.makeDeviceCodeRequest(a.ctx, discovery)
		if err != nil {
			return authCodeMsg{err: errors.Wrap(err, "failed to request a device code")}
		}
		return authCodeMsg{code: code, discovery: discovery}
	}
}

func (a *wizardAuth) awaitToken(code *deviceCodeResponse, discovery *openIDDiscoveryResponse) tea.Cmd {
	return func() tea.Msg {
		resp, err := a.client.authFlow.waitForUser(a.ctx, code, discovery)
		if err != nil {
			return authDoneMsg{err: err}
		}
		t, err := buildToken(resp, discovery.TokenEndPoint, a.client.authFlow.clientID)
		if err != nil {
			return authDoneMsg{err: err}
		}
		a.client.conf.Auth = t
		if err := storeConfigToCache(a.client.conf); err != nil {
			return authDoneMsg{err: err}
		}
		// Dial with the new token so the org lookup that follows generation is ready to go.
		if err := a.client.ensureLoggedIn(a.ctx); err != nil {
			return authDoneMsg{err: err}
		}
		return authDoneMsg{email: t.User.Email}
	}
}

func openBrowserCmd(rawURL string) tea.Cmd {
	return func() tea.Msg {
		// The wizard always shows the URL, so failing to launch a browser is not worth
		// reporting: the user can still finish the flow by hand.
		utils.UncheckedError(openbrowser(rawURL))
		return browserOpenedMsg{}
	}
}

// noteMarkupEscaper escapes the characters huh renders as inline markdown inside a note
// description. Without it a verification URL such as ".../device?user_code=ABCD" loses its
// underscore to an italic run, leaving the user a URL that does not work.
var noteMarkupEscaper = strings.NewReplacer(`\`, `\\`, "_", `\_`, "*", `\*`, "`", "\\`")

// describe is the body of the wizard's sign-in page.
func (a *wizardAuth) describe() string {
	switch a.phase {
	case authSignedIn:
		return fmt.Sprintf("Signed in as %s.", noteMarkupEscaper.Replace(a.email))
	case authRequestingCode:
		return "Asking Viam for a device code..."
	case authAwaitingBrowser:
		return fmt.Sprintf("Finish signing in at\n  %s\nand check that the code shown there is %s.\n\n"+
			"You can keep filling out the rest of this form while that happens.",
			noteMarkupEscaper.Replace(a.verifyURL), noteMarkupEscaper.Replace(a.userCode))
	case authFailed:
		return fmt.Sprintf("Sign-in failed: %s\n\nSelect \"Sign in\" to try again.",
			noteMarkupEscaper.Replace(a.err.Error()))
	case authSignedOut:
		fallthrough
	default:
		return "Registering with Viam associates the project with your organization, which\n" +
			"requires an account.\n\n" +
			"Selecting \"Sign in\" opens your browser. This form stays active while you finish."
	}
}

// status is the one-line banner the wizard keeps under the form, so sign-in progress stays
// visible after the user has tabbed on to the next page.
func (a *wizardAuth) status(s wizardStyles, spin bubblespinner.Model) string {
	switch a.phase {
	case authSignedIn:
		return s.good.Render("✓ signed in") + s.hint.Render(" as "+a.email)
	case authRequestingCode:
		return spin.View() + s.hint.Render("contacting Viam...")
	case authAwaitingBrowser:
		line := spin.View() + s.hint.Render("waiting for browser sign-in at ") + s.link.Render(a.verifyURL)
		if a.userCode != "" {
			line += s.hint.Render(" (code " + a.userCode + ")")
		}
		return line
	case authFailed:
		return s.bad.Render("✗ sign-in failed: ") + s.hint.Render(a.err.Error())
	case authSignedOut:
		fallthrough
	default:
		return s.hint.Render("not signed in")
	}
}

// signedInEmail is the email of the currently signed-in user, or "" when the CLI is
// authenticated with an API key rather than a user login.
func signedInEmail(client *viamClient) string {
	if t, ok := client.conf.Auth.(*token); ok {
		return t.User.Email
	}
	return ""
}

// generateWizard is the bubbletea model that hosts the huh form. Hosting the form, rather
// than calling form.Run, is what lets sign-in happen in the background: tea commands keep
// delivering messages while the user carries on answering questions.
type generateWizard struct {
	formNav
	form   *huh.Form
	args   *moduleArgs
	auth   *wizardAuth
	spin   bubblespinner.Model
	styles wizardStyles
	// resource is the "<subtype> <type>" pair the resource picker binds to, e.g. "arm component".
	resource string
	finished bool
}

func newGenerateWizard(ctx context.Context, args *moduleArgs, client *viamClient, signedIn bool) *generateWizard {
	// Terminals are dark far more often than not; a BackgroundColorMsg corrects this on the
	// first update if the terminal reports otherwise.
	styles := newWizardStyles(true)
	auth := &wizardAuth{client: client, ctx: ctx}
	if signedIn {
		auth.phase = authSignedIn
		auth.email = signedInEmail(client)
	}
	w := &generateWizard{
		args:     args,
		auth:     auth,
		spin:     bubblespinner.New(bubblespinner.WithSpinner(bubblespinner.MiniDot), bubblespinner.WithStyle(styles.hint)),
		styles:   styles,
		resource: canonicalResource(args.ModelArgs.ResourceSubtype),
	}
	w.form = huh.NewForm(
		w.targetGroup(),
		w.detailsGroup(),
		w.signInGroup(),
		w.modelGroup(),
		w.appGroup(),
	).WithWidth(wizardWidth).WithTheme(huh.ThemeFunc(huh.ThemeCharm))
	return w
}

func (w *generateWizard) targetGroup() *huh.Group {
	return huh.NewGroup(
		huh.NewNote().
			Title("Create a new Viam project").
			Description("Answer a few questions and the CLI will scaffold the project for you.\n"+
				"Move between fields and pages with tab and shift+tab; quit with ctrl+c."),
		huh.NewSelect[generateTarget]().
			Title("What would you like to build?").
			DescriptionFunc(func() string {
				switch w.args.Target {
				case targetModule:
					return "A modular resource that adds custom components or services to your machine."
				case targetApp:
					return "A web application that connects to your machine via the Viam SDK."
				case targetModuleApp:
					return "A Go modular resource and a web application in the same module."
				default:
					return ""
				}
			}, &w.args.Target).
			Options(
				huh.NewOption("Module", targetModule),
				huh.NewOption("App", targetApp),
				huh.NewOption("Module + App", targetModuleApp),
			).
			Value(&w.args.Target),
	).Title("Project type")
}

func (w *generateWizard) detailsGroup() *huh.Group {
	return huh.NewGroup(
		huh.NewInput().
			Title("Name").
			Description("Names the generated directory and the registry entry.").
			Placeholder("my-module").
			Value(&w.args.Name).
			Validate(w.whenAdvancing(func(s string) error {
				if err := validateName("project", s); err != nil {
					return err
				}
				if _, err := os.Stat(s); err == nil {
					return errors.New("a directory with this name already exists")
				}
				return nil
			})),
		huh.NewSelect[string]().
			Title("Language").
			DescriptionFunc(func() string {
				if w.args.Target.includesApp() {
					return "Projects that ship an app are generated as Go modules."
				}
				return "The Viam SDK your module will be written against."
			}, &w.args.Target).
			OptionsFunc(func() []huh.Option[string] {
				if w.args.Target.includesApp() {
					return []huh.Option[string]{huh.NewOption("Go", golang)}
				}
				return []huh.Option[string]{
					huh.NewOption("Python", python),
					huh.NewOption("Go", golang),
					huh.NewOption("C++", cpp),
				}
			}, &w.args.Target).
			Value(&w.args.Language),
		huh.NewSelect[moduleVisibility]().
			Title("Visibility").
			Description("Private projects are only visible to members of your organization.").
			Options(
				huh.NewOption("Private", visibilityPrivate),
				huh.NewOption("Public", visibilityPublic),
				huh.NewOption("Public, unlisted", visibilityPublicUnlisted),
			).
			Value(&w.args.Visibility),
		huh.NewInput().
			Title("Namespace or organization ID").
			Description("Your organization's public namespace, or its UUID.").
			Placeholder("my-namespace").
			Value(&w.args.Namespace).
			Validate(w.whenAdvancing(func(s string) error {
				if strings.TrimSpace(s) == "" {
					return errors.New("namespace or organization ID must not be empty")
				}
				return nil
			})),
		huh.NewConfirm().
			Title("Register with Viam").
			DescriptionFunc(func() string {
				if w.auth.signedIn() {
					return fmt.Sprintf("Creates the project in your organization as %s, so you can\n"+
						"upload versions to it. Decline to generate a local-only project.", w.auth.email)
				}
				return "Creates the project in your organization so you can upload versions to it.\n" +
					"You are not signed in yet; the next page can sign you in."
			}, &w.auth.phase).
			Value(&w.args.ShouldRegister),
		huh.NewConfirm().
			Title("Dry run").
			Description("List the files this would write, and the command that skips this form.").
			Affirmative("Dry run").
			Negative("Generate").
			Value(&w.args.DryRun),
	).Title("Project details")
}

func (w *generateWizard) signInGroup() *huh.Group {
	return huh.NewGroup(
		huh.NewNote().
			TitleFunc(func() string {
				if w.auth.signedIn() {
					return "Signed in to Viam"
				}
				return "Sign in to Viam"
			}, &w.auth.phase).
			DescriptionFunc(w.auth.describe, &w.auth.phase),
		huh.NewConfirm().
			TitleFunc(func() string {
				switch {
				case w.auth.signedIn():
					return "Signed in; continue"
				case w.auth.pending():
					return "Signing in..."
				case w.auth.phase == authFailed:
					return "Try signing in again?"
				default:
					return "Open your browser to sign in?"
				}
			}, &w.auth.phase).
			Affirmative("Sign in").
			Negative("Not now").
			Value(&w.auth.requested),
	).Title("Authentication").
		WithHideFunc(func() bool { return !w.args.ShouldRegister || w.auth.signedIn() })
}

func (w *generateWizard) modelGroup() *huh.Group {
	options := buildResourceOptions()
	return huh.NewGroup(
		huh.NewSelect[string]().
			Title("Resource").
			Description("The component or service API your model implements.\n"+
				"Scroll with the arrow keys, and filter with `/`.").
			Options(options...).
			Height(min(len(options), 10)).
			Value(&w.resource),
		huh.NewInput().
			Title("Model name").
			Description("The name of the resource model your module will provide.").
			PlaceholderFunc(func() string { return defaultModelName(w.resource) }, &w.resource).
			SuggestionsFunc(func() []string { return []string{defaultModelName(w.resource)} }, &w.resource).
			Value(&w.args.ModelArgs.ModelName).
			Validate(w.whenAdvancing(func(s string) error { return validateName("model", s) })),
	).Title("Model").
		WithHideFunc(func() bool { return !w.args.Target.includesModule() })
}

func (w *generateWizard) appGroup() *huh.Group {
	return huh.NewGroup(
		huh.NewInput().
			Title("App name").
			Description("The name of the web app entry in your project's meta.json.").
			PlaceholderFunc(func() string { return w.args.Name }, &w.args.Name).
			Value(&w.args.AppArgs.AppName).
			Validate(w.whenAdvancing(func(s string) error { return validateName("app", s) })),
		huh.NewSelect[string]().
			Title("App type").
			Description("Single machine apps connect to one machine.\n"+
				"Multi machine apps can connect to several machines in your fleet.").
			Options(
				huh.NewOption("Single machine", appTypeSingleMachine),
				huh.NewOption("Multi machine", appTypeMultiMachine),
			).
			Value(&w.args.AppArgs.AppType),
	).Title("App").
		WithHideFunc(func() bool { return !w.args.Target.includesApp() })
}

// Init implements tea.Model.
func (w *generateWizard) Init() tea.Cmd { return w.form.Init() }

// Update implements tea.Model.
func (w *generateWizard) Update(msg tea.Msg) (tea.Model, tea.Cmd) {
	var cmds []tea.Cmd

	switch msg := msg.(type) {
	case tea.KeyPressMsg:
		w.noteKey(msg)
	case tea.BackgroundColorMsg:
		w.styles = newWizardStyles(msg.IsDark())
		w.spin.Style = w.styles.hint
	case bubblespinner.TickMsg:
		// Let the ticks lapse once there is nothing left to wait on.
		if w.auth.pending() {
			var cmd tea.Cmd
			w.spin, cmd = w.spin.Update(msg)
			cmds = append(cmds, cmd)
		}
	case authCodeMsg:
		if msg.err != nil {
			w.auth.fail(msg.err)
			break
		}
		w.auth.phase = authAwaitingBrowser
		w.auth.verifyURL = msg.code.VerificationURIComplete
		w.auth.userCode = msg.code.UserCode
		cmds = append(cmds, w.auth.awaitToken(msg.code, msg.discovery), openBrowserCmd(w.auth.verifyURL))
	case authDoneMsg:
		if msg.err != nil {
			w.auth.fail(msg.err)
			break
		}
		w.auth.phase = authSignedIn
		w.auth.email = msg.email
	}

	form, cmd := w.form.Update(msg)
	if f, ok := form.(*huh.Form); ok {
		w.form = f
	}
	cmds = append(cmds, cmd)

	if w.auth.requested && (w.auth.phase == authSignedOut || w.auth.phase == authFailed) {
		w.auth.phase = authRequestingCode
		w.auth.err = nil
		cmds = append(cmds, w.auth.requestCode(), w.spin.Tick)
	}

	// Submitting while a sign-in is still in flight would fail to resolve the organization, so
	// hold the program open until the device flow settles.
	settled := w.form.State == huh.StateAborted || (w.form.State == huh.StateCompleted && !w.auth.pending())
	if settled && !w.finished {
		w.finished = true
		cmds = append(cmds, tea.Quit)
	}
	return w, tea.Batch(cmds...)
}

// View implements tea.Model.
func (w *generateWizard) View() tea.View {
	if w.finished {
		return tea.NewView("")
	}
	body := w.form.View()
	if w.form.State == huh.StateCompleted && w.auth.pending() {
		body = "  " + w.spin.View() + w.styles.hint.Render("finishing sign-in before generating...")
	}
	parts := []string{w.styles.header.Render("viam module generate"), "", body}
	if w.args.ShouldRegister || w.auth.phase != authSignedOut {
		parts = append(parts, w.auth.status(w.styles, w.spin))
	}
	return tea.NewView(lipgloss.JoinVertical(lipgloss.Left, parts...))
}

// syncBuffer collects what the CLI's helpers print while the wizard owns the terminal. It is
// guarded because a sign-in still in flight when the wizard exits can write from its own
// goroutine after the main one has moved on.
type syncBuffer struct {
	mu  sync.Mutex
	buf bytes.Buffer
}

func (b *syncBuffer) Write(p []byte) (int, error) {
	b.mu.Lock()
	defer b.mu.Unlock()
	return b.buf.Write(p)
}

func (b *syncBuffer) String() string {
	b.mu.Lock()
	defer b.mu.Unlock()
	return b.buf.String()
}

// runGenerateWizard runs the wizard and writes the user's answers into args. It reports
// whether the user is signed in by the time the wizard exits, which may differ from when it
// started because the wizard can sign them in.
func runGenerateWizard(
	ctx context.Context, cmd *cli.Command, args *moduleArgs, client *viamClient, signedIn bool,
) (bool, error) {
	// Quitting mid-sign-in leaves the device-code poller running; cancelling here stops it
	// rather than letting it keep polling until the code expires.
	wizardCtx, cancel := context.WithCancel(ctx)
	defer cancel()

	final, err := runTUI(wizardCtx, cmd, newGenerateWizard(wizardCtx, args, client, signedIn))
	if err != nil {
		return signedIn, errors.Wrap(err, "encountered an error running the generate wizard")
	}

	done, ok := final.(*generateWizard)
	if !ok {
		return signedIn, errors.New("generate wizard returned an unexpected model")
	}
	if done.form.State != huh.StateCompleted {
		return signedIn, huh.ErrUserAborted
	}
	if args.Target.includesModule() {
		args.ModelArgs.ResourceSubtype = flagSubtype(done.resource)
	}
	if err := args.normalize(); err != nil {
		return signedIn, err
	}
	if done.auth.phase == authFailed {
		warningf(cmd.Root().ErrWriter, "sign-in did not complete: %v", done.auth.err)
	}
	return done.auth.signedIn(), nil
}

// canonicalResource maps a --resource-subtype value onto the "<subtype> <type>" pair the
// resource picker binds to, so a flag-supplied subtype shows up already selected.
func canonicalResource(subtype string) string {
	inputs := modulegen.ModuleInputs{ResourceSubtype: subtype}
	if subtype == "" || inputs.CheckResourceAndSetType() != nil {
		return modulegen.Resources[0]
	}
	return inputs.Resource
}

// flagSubtype turns a picker value back into a --resource-subtype value. Generic needs its
// resource type appended, because "generic" on its own is ambiguous.
func flagSubtype(resource string) string {
	fields := strings.Fields(resource)
	if len(fields) < 2 {
		return resource
	}
	if fields[0] == "generic" {
		return "generic-" + fields[1]
	}
	return fields[0]
}

// defaultModelName suggests a model name for the selected resource.
func defaultModelName(resource string) string {
	fields := strings.Fields(resource)
	if len(fields) == 0 {
		return ""
	}
	name := fields[0]
	if name == "generic" && len(fields) > 1 {
		name += "_" + fields[1]
	}
	return "my_" + name
}

// plannedFile is one entry in the dry run's file summary.
type plannedFile struct {
	path    string
	updated bool
}

// templateOutputs lists the paths a template directory renders to, relative to the project
// root, applying the same prefix-stripping and per-OS filtering the generator applies.
func templateOutputs(name string) ([]string, error) {
	sub, err := fs.Sub(templates, path.Join(templatesPath, name))
	if err != nil {
		return nil, err
	}
	var out []string
	err = fs.WalkDir(sub, ".", func(filePath string, d fs.DirEntry, err error) error {
		switch {
		case err != nil:
			return err
		case d.IsDir():
			return nil
		case strings.HasPrefix(d.Name(), templatePrefix):
			out = append(out, strings.ReplaceAll(filePath, templatePrefix, ""))
		case runtime.GOOS == osWindows && strings.HasSuffix(d.Name(), ".sh"):
		case runtime.GOOS != osWindows && strings.HasSuffix(d.Name(), ".bat"):
		default:
			out = append(out, filePath)
		}
		return nil
	})
	return out, err
}

// stubOutputs lists the files the language-specific stub generators write.
func stubOutputs(inputs modulegen.ModuleInputs) []string {
	switch inputs.Language {
	case golang:
		return []string{"module.go"}
	case python:
		return []string{path.Join("src", "models", inputs.ModelSnake+".py")}
	case cpp:
		return []string{
			"main.cpp",
			path.Join("src", inputs.ModelSnake+".cpp"),
			path.Join("src", inputs.ModelSnake+".hpp"),
			"CMakeLists.txt",
			"conanfile.py",
			"conan.lock",
		}
	default:
		return nil
	}
}

// planGeneratedFiles lists everything a real run would write, by walking the same embedded
// templates the generator renders from.
func planGeneratedFiles(args moduleArgs, inputs *modulegen.ModuleInputs) ([]plannedFile, error) {
	var files []plannedFile
	seen := map[string]int{}
	add := func(updated bool, parts ...string) {
		p := filepath.Join(append([]string{args.Name}, parts...)...)
		if idx, ok := seen[p]; ok {
			// A file this run creates and then edits is, to the user, simply created.
			files[idx].updated = files[idx].updated && updated
			return
		}
		seen[p] = len(files)
		files = append(files, plannedFile{path: p, updated: updated})
	}

	if args.Target.includesModule() {
		add(false, defaultManifestFilename)
		add(false, ".viam-gen-info")
		add(false, defaultReadmeFilename)
		add(false, inputs.ModelReadmeLink)
		workflows, err := templateOutputs(".github")
		if err != nil {
			return nil, err
		}
		for _, f := range workflows {
			add(false, ".github", f)
		}
		// The C++ templates live in the C++ SDK repo rather than here; its stubs cover them.
		if inputs.Language != cpp {
			langFiles, err := templateOutputs(inputs.Language)
			if err != nil {
				return nil, err
			}
			for _, f := range langFiles {
				add(false, f)
			}
		}
		for _, f := range stubOutputs(*inputs) {
			add(false, f)
		}
	}

	if args.Target == targetApp {
		add(false, defaultManifestFilename)
		appFiles, err := templateOutputs("app")
		if err != nil {
			return nil, err
		}
		for _, f := range appFiles {
			add(false, f)
		}
	}

	if args.Target == targetModuleApp {
		add(false, "webapp.go")
		add(false, "auth.js")
		add(false, path.Join("dist", "index.html"))
		add(true, path.Join("cmd", "module", "main.go"))
		add(true, "Makefile")
		add(true, defaultManifestFilename)
	}

	slices.SortFunc(files, func(a, b plannedFile) int { return strings.Compare(a.path, b.path) })
	return files, nil
}

var namespaceCleanRegexp = regexp.MustCompile(`[^a-zA-Z0-9_-]+`)

// printGenerationPlan renders the dry run: the files a real run would write, and the command
// that reproduces this run without the wizard.
func printGenerationPlan(cmd *cli.Command, args moduleArgs) error {
	inputs, err := args.toModuleInputs()
	if err != nil {
		return err
	}
	if !args.ShouldRegister {
		// Mirror wrapResolveOrg, which cleans a local-only namespace before it reaches the
		// generated file names.
		inputs.Namespace = namespaceCleanRegexp.ReplaceAllString(inputs.Namespace, "")
	}
	if args.Target.includesModule() {
		populateAdditionalInfo(inputs)
	}
	files, err := planGeneratedFiles(args, inputs)
	if err != nil {
		return err
	}

	w := cmd.Root().Writer
	printf(w, "Dry run: nothing was written.")
	printf(w, "")
	printf(w, "%d files would be written (+ created, ~ updated):", len(files))
	for _, f := range files {
		marker := "+"
		if f.updated {
			marker = "~"
		}
		printf(w, "  %s %s", marker, f.path)
	}
	if args.ShouldRegister {
		printf(w, "")
		printf(w, "A %s entry named %q would be created in %s on app.viam.com.",
			args.Visibility, args.Name, args.Namespace)
	}
	printf(w, "")
	printf(w, "Run this to generate the same project without the form:")
	printf(w, "  %s", args.rerunCommand())
	return nil
}

// generateStep is one unit of scaffolding work, titled so the spinner can narrate it.
type generateStep struct {
	title string
	run   func() error
	// optional marks a step whose failure still leaves a usable, if incomplete, project.
	optional bool
}

// runGenerateSteps runs steps in order behind a spinner, falling back to plain logging when
// there is no terminal to animate, or when debug output would fight the spinner for the line.
// It returns the failures of optional steps separately from the one that stopped the run.
func runGenerateSteps(cmd *cli.Command, debug bool, steps []generateStep) ([]error, error) {
	var warnings []error
	var fatal error
	run := func(report func(string)) {
		for _, step := range steps {
			report(step.title)
			err := step.run()
			switch {
			case err == nil:
			case step.optional:
				warnings = append(warnings, err)
			default:
				fatal = err
				return
			}
		}
	}

	if debug || !isInteractive() {
		run(func(title string) { printf(cmd.Root().Writer, "%s", title) })
		return warnings, fatal
	}

	s := spinner.New()
	s.Action(func() { run(func(title string) { s.Title(title) }) })
	if err := s.Run(); err != nil {
		return warnings, err
	}
	return warnings, fatal
}

// generationResult collects the parts of a run that are only known once it has happened.
type generationResult struct {
	registryURL string
	// createdDir records that this run made the project directory, so a failure part way
	// through can clean it up without ever deleting a directory the user already had.
	createdDir bool
}

// runGeneration scaffolds the project described by args.
func runGeneration(ctx context.Context, cmd *cli.Command, c *viamClient, args moduleArgs) error {
	gArgs, err := getGlobalArgs(cmd)
	if err != nil {
		return err
	}
	globalArgs := *gArgs

	inputs, err := args.toModuleInputs()
	if err != nil {
		return err
	}
	if args.Target.includesModule() {
		if err := checkLanguageVersion(inputs.Language); err != nil {
			return err
		}
	}
	// Resolving the org rewrites Namespace, which the generated file names embed, so it has to
	// happen before populateAdditionalInfo derives them.
	if err := wrapResolveOrg(ctx, c, inputs); err != nil {
		return err
	}
	if args.Target.includesModule() {
		populateAdditionalInfo(inputs)
	}

	result := &generationResult{}
	var steps []generateStep
	switch args.Target {
	case targetModule:
		steps = moduleGenerationSteps(ctx, cmd, c, inputs, globalArgs, result)
	case targetApp:
		steps = appGenerationSteps(ctx, cmd, c, args, inputs, globalArgs, result)
	case targetModuleApp:
		steps = append(moduleGenerationSteps(ctx, cmd, c, inputs, globalArgs, result), attachAppSteps(args, inputs)...)
	default:
		return errors.Errorf("invalid generate type %q", args.Target)
	}

	warnings, err := runGenerateSteps(cmd, globalArgs.Debug, steps)
	if err != nil {
		// A half-written directory is worse than none, since the next run would fail on mkdir.
		if result.createdDir {
			if rmErr := os.RemoveAll(args.Name); rmErr != nil {
				return errors.Wrapf(err, "generation failed, and the incomplete project at %s could not be removed", args.Name)
			}
		}
		return errors.Wrap(err, "unable to generate project")
	}
	for _, warning := range warnings {
		warningf(cmd.Root().ErrWriter, "%s", warning)
	}
	printGenerationSummary(cmd, args, result, len(warnings) > 0)
	return nil
}

// sdkVersionFromTag turns a GitHub release tag into a bare semver. The Go and Python SDKs tag
// releases as "vX.Y.Z"; the C++ SDK tags them as "release/vX.Y.Z".
func sdkVersionFromTag(tag string) string {
	if idx := strings.LastIndex(tag, "/"); idx != -1 {
		tag = tag[idx+1:]
	}
	return strings.TrimPrefix(tag, "v")
}

func moduleGenerationSteps(
	ctx context.Context, cmd *cli.Command, c *viamClient,
	inputs *modulegen.ModuleInputs, globalArgs globalArgs, result *generationResult,
) []generateStep {
	return []generateStep{
		{title: "Getting the latest SDK release...", run: func() error {
			// Templates ship with this CLI, but the stubs are generated against the SDK the
			// user will actually build with, so the current release tag decides the version.
			tag, err := getLatestSDKTag(ctx, cmd, inputs.Language, globalArgs)
			if err != nil {
				return err
			}
			inputs.SDKVersion = sdkVersionFromTag(tag)
			return nil
		}},
		{title: "Setting up the module directory...", run: func() error {
			if err := setupDirectories(cmd, inputs.ModuleName, globalArgs); err != nil {
				return err
			}
			result.createdDir = true
			return nil
		}},
		{title: "Creating the module and its manifest...", run: func() error {
			url, err := createModuleAndManifest(ctx, cmd, c, *inputs, globalArgs)
			result.registryURL = url
			return err
		}},
		{title: "Rendering common files...", run: func() error {
			return renderCommonFiles(cmd, *inputs, globalArgs)
		}},
		{title: fmt.Sprintf("Copying %s files...", inputs.Language), run: func() error {
			return copyLanguageTemplate(cmd, inputs.Language, inputs.ModuleName, globalArgs)
		}},
		{title: "Rendering templates...", run: func() error {
			return renderTemplate(cmd, *inputs, globalArgs)
		}},
		{title: fmt.Sprintf("Generating %s stubs...", inputs.Language), optional: true, run: func() error {
			return generateStubs(cmd, *inputs, globalArgs)
		}},
	}
}

func appGenerationSteps(
	ctx context.Context, cmd *cli.Command, c *viamClient, args moduleArgs,
	inputs *modulegen.ModuleInputs, globalArgs globalArgs, result *generationResult,
) []generateStep {
	app := &appInputs{AppName: args.AppArgs.AppName, AppType: args.AppArgs.AppType}
	data := appTemplateData{
		ModuleName:      inputs.ModuleName,
		ModuleLowercase: strings.ReplaceAll(strings.ToLower(inputs.ModuleName), "-", ""),
		AppName:         app.AppName,
		AppType:         app.AppType,
		Namespace:       inputs.Namespace,
		Visibility:      inputs.Visibility,
		ConfigName:      "Config",
	}
	return []generateStep{
		{title: "Setting up the app directory...", run: func() error {
			if err := setupDirectories(cmd, inputs.ModuleName, globalArgs); err != nil {
				return err
			}
			result.createdDir = true
			return nil
		}},
		{title: "Registering the app with Viam...", run: func() error {
			if !inputs.RegisterOnApp {
				return nil
			}
			resp, err := c.createModule(ctx, inputs.ModuleName, inputs.OrgID)
			if err != nil {
				return errors.Wrap(err, "failed to register app")
			}
			result.registryURL = resp.GetUrl()
			return nil
		}},
		{title: "Generating the manifest...", run: func() error {
			id := moduleID{prefix: inputs.Namespace, name: inputs.ModuleName}
			return renderManifest(cmd, id.String(), *inputs, globalArgs, app)
		}},
		{title: "Copying app files...", run: func() error {
			return copyLanguageTemplate(cmd, "app", inputs.ModuleName, globalArgs)
		}},
		{title: "Rendering app templates...", run: func() error {
			return renderAppTemplate(cmd, inputs.ModuleName, data, globalArgs)
		}},
		{title: "Rendering the landing page...", optional: true, run: func() error {
			return renderAppLandingPage(inputs.ModuleName, app.AppName)
		}},
	}
}

// attachAppSteps adds a web app to a module that the module steps have just generated.
func attachAppSteps(args moduleArgs, inputs *modulegen.ModuleInputs) []generateStep {
	dir := inputs.ModuleName
	app := &appInputs{AppName: args.AppArgs.AppName, AppType: args.AppArgs.AppType}
	data := appTemplateData{
		ModuleName:      inputs.ModuleName,
		ModuleLowercase: strings.ReplaceAll(strings.ToLower(inputs.ModuleName), "-", ""),
		AppName:         app.AppName,
		AppType:         app.AppType,
		Namespace:       inputs.Namespace,
		Visibility:      inputs.Visibility,
		// The module already declares a Config for its model, so the webapp's config type
		// needs a name of its own.
		ConfigName: "WebappConfig",
	}
	return []generateStep{
		{title: "Generating the webapp component...", run: func() error {
			if err := addGoWebappFile(dir, data); err != nil {
				return errors.Wrap(err, "failed to generate webapp.go")
			}
			return addGoWebappToMain(filepath.Join(dir, "cmd", "module", "main.go"), data)
		}},
		{title: "Setting up app files...", run: func() error {
			if err := addAppStaticFiles(dir); err != nil {
				return errors.Wrap(err, "failed to set up app files")
			}
			return updateMakefileForApp(filepath.Join(dir, "Makefile"))
		}},
		{title: "Adding the app to meta.json...", run: func() error {
			return addAppToManifest(filepath.Join(dir, defaultManifestFilename), app, data)
		}},
	}
}

// renderAppLandingPage rewrites dist/index.html with the rendered README, so a freshly
// generated app serves its own setup instructions until a real frontend replaces it.
func renderAppLandingPage(dir, appName string) error {
	readme, err := os.ReadFile(filepath.Join(dir, defaultReadmeFilename)) //nolint:gosec
	if err != nil {
		return err
	}
	const style = "body{max-width:800px;margin:40px auto;padding:0 20px;}" +
		"pre{background:#f4f4f4;padding:12px;border-radius:4px;overflow-x:auto;}"
	page := fmt.Sprintf("<!DOCTYPE html>\n<html><head><meta charset=\"UTF-8\"><title>%s</title>\n"+
		"<style>%s</style>\n</head><body>%s</body></html>", appName, style, blackfriday.Run(readme))
	//nolint:gosec
	return os.WriteFile(filepath.Join(dir, "dist", "index.html"), []byte(page), 0o644)
}

func targetLabel(t generateTarget) string {
	switch t {
	case targetApp:
		return "App"
	case targetModuleApp:
		return "Module and app"
	case targetModule:
		fallthrough
	default:
		return "Module"
	}
}

func printGenerationSummary(cmd *cli.Command, args moduleArgs, result *generationResult, partial bool) {
	w := cmd.Root().Writer
	cwd, err := os.Getwd()
	if err != nil {
		cwd = "."
	}
	dir := filepath.Join(cwd, args.Name)

	if partial {
		warningf(cmd.Root().ErrWriter, "some steps did not complete; the project at %s may be incomplete", dir)
	}
	printf(w, "%s successfully generated at %s", targetLabel(args.Target), dir)
	if result.registryURL != "" {
		printf(w, "You can view it here: %s", result.registryURL)
	}
	if args.Target.includesApp() {
		printAppBuildNextSteps(w, filepath.Join(dir, defaultReadmeFilename))
	}
	if runtime.GOOS == osWindows && args.Language == python {
		printf(w, "Python modules generated for Windows do not have cloud build support yet\n"+
			"You can test locally and then use `viam module upload` to manually upload,\n"+
			"but the uploaded module will only work on Windows.\n"+
			"To access your module in app, make sure to add \"tcp_mode\": true to the module config json\n"+
			"and set your local module's executable path to run.bat")
	}
}
