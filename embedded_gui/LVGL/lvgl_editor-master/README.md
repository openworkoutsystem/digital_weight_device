# LVGL Pro

**LVGL Pro is a complete toolkit to build, test, share and ship embedded UIs efficiently.**

It consists of four tightly related tools:

1. **XML Editor**: The heart of LVGL Pro. A desktop app to build components and screens in XML, manage data bindings, translations, animations, tests, and more. Learn more about the [XML Format](https://docs.lvgl.io/master/details/xml/xml/index.html) and the [Editor](https://docs.lvgl.io/master/details/xml/editor/index.html).  
2. **Online Viewer**: Run the Editor in your browser, open GitHub projects, and share easily without setting up a developer environment. Visit [https://viewer.lvgl.io](https://viewer.lvgl.io).  
3. **CLI Tool**: Generate C code and run tests in CI/CD. See the details [here](https://docs.lvgl.io/master/details/xml/tools/cli.html).  
4. **Figma Plugin**: Sync and extract styles directly from Figma. See how it works [here](https://docs.lvgl.io/master/details/xml/tools/figma.html).

Together, these tools let developers build UIs efficiently, test them reliably, and collaborate with team members and customers.

## Key Features

- **Component‑oriented**: Create reusable UI components with custom API
- **XML‑based**: Describe components in a human‑readable, HTML‑like syntax  
- **Runtime XML loading**: Load or update UI components from XML at runtime without recompiling  
- **Instant preview**: See changes immediately as you edit  
- **Inspector**: Visualise padding, marginf, click area,and so on and move/resize UI elements
- **Animations**: Powerful animations organized into timelines, played on any events.  
- **Figma support**: Extract styles from Figma with the LVGL plugin  
- **Share online**: CI pipelines can generate a live preview website from your XML  
- **Custom widget creation**: Add C‑based widgets by recompiling the editor  
- **C export**: Export both components and widgets to C code  
- **Built-in testing**: Create UI tests quickly and run them automatically  
- **Translations**: Manage text translations and localization flexibly  
- **Data binding**: Bind widgets to global or application data, bridging UI and app logic  

## Why XML?

Using XML instead of only drag‑and‑drop editing has several advantages:

- **Familiar**: syntax is HTML‑like, easy to learn and readable  
- **Version control friendly**: plain text with human‑readable diffs, no binary files  
- **Easy to share**: copy, paste, and send as text  
- **Reusable patterns**: copy and reuse snippets across projects  
- **Automation ready**: scripts and CI/CD systems can process XML effortlessly  
- **AI compatible**: AI tools can parse, generate, and refactor XML  
- **Modular by design**: easily compose UIs from smaller components  
- **Fast to edit**: quicker to type or refactor than dragging UI elements  
- **Runtime loading**: parse XML on target devices without recompiling  
- **Cross-platform**: the same XML works across all LVGL targets  

## The Workflow

1. **Create reusable components** in the XML Editor. Use the pixel‑perfect, instant preview to see how the UI renders live.  
2. Build screens from these components, then add UI tests, transitions, data bindings, and translations directly in XML.  
3. If you need custom logic or widgets, write them in C, then recompile the editor’s preview to include your code—so your custom widgets run both in preview and target firmware.  
4. Use the Figma plugin to import style properties from Figma or even synchronize them automatically.  
5. Integrate the developed UI into your project by:
   - **Exporting LVGL C code** from your XML → see details [here](https://docs.lvgl.io/master/details/xml/integration/c_code.html)  
   - **Or loading the XML at runtime** on device → see details [here](https://docs.lvgl.io/master/details/xml/integration/xml.html)  
6. When you push to a GitHub repo, use the Viewer to browse, view, and edit the UI in a browser—no local setup is required for your teammate, designer, or manager.  
7. In CI/CD, use the CLI tool to export C code and run UI tests as part of your pipeline.

## Get Started

### In your browser

Open the online viewer: [https://viewer.lvgl.io/](https://viewer.lvgl.io/)  
Then pick an example or tutorial from the list to explore.

### Locally

1. Download the LVGL Pro Editor (for Windows, Linux, or macOS) or install the VSCode extension (coming soon) from https://pro.lvgl.io  
2. Clone this repository  
3. Launch the editor and open one of the XML examples or tutorials  

## Licenses

See https://pro.lvgl.io/pricing for the avaialable subscription plans.


## Contribution

We’re building this tool for you. Your feedback drives its development.  

Please open an issue to share suggestions, report bugs, or request features.  

Thank you! ❤️
