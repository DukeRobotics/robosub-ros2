# Foxglove Custom Extensions
This monorepo contains our custom Foxglove development tools and extensions.

<img width="1440" alt="Foxglove Studio" src="https://github.com/user-attachments/assets/36e191b0-6a70-493d-8c44-6dd849ab425f" />

## Setup
For initial setup instructions, follow the root [SETUP.md](../SETUP.md#set-up-foxglove-optional) file.

## Using foxglove.py
> [!IMPORTANT]
> The `foxglove.py` CLI should be run *inside* the Docker container (even on your local machine). For more details see [SETUP.md](../SETUP.md#set-up-the-docker-container).

The [`foxglove.py`](foxglove.py) CLI provides a unified interface to manage our custom extensions. It integrates commands from the [`foxglove`](https://github.com/foxglove/foxglove-cli), [`foxglove-extension`](https://github.com/foxglove/create-foxglove-extension/blob/main/src/bin/foxglove-extension.ts), and [`npm`](https://docs.npmjs.com/cli/v11/commands) CLIs. Extensions can be installed locally or published to our organization.

To run the CLI you can use `python3 foxglove.py` (inside of the `foxglove` directory) or its alias `fox` (declared in `ros_bashrc.sh`).


### Quick Start
There are two steps in the installation process:

1. **Build:** Prepares an extension for installation or publishing.
2. **Install:** Locally installs an extension.

Therefore, to install all extensions, run:
```bash
fox build
fox install
```

To clean up the Foxglove monorepo and uninstall local extensions, run:
```bash
fox clean
fox uninstall
```
*Clean* and *uninstall* will revert the changes made by *build* and *install* respectively.

### User Guide
```bash
fox [SUBCOMMAND]
```
- [`b, build`](#build) - Prepare an extension for installation or publishing.
- [`i, install`](#install) - Install an extension locally.
- [`w, watch`](#watch) - Watch an extension for changes and automatically reinstall.
- [`p, publish`](#publish) - Publish an extension to your organization.
- [`c, clean`](#clean) - Remove build files from the Foxglove monorepo.
- [`u, uninstall`](#uninstall) - Uninstall a locally installed extension.
- [`d, doctor`](#doctor) - Troubleshoot the Foxglove development environment.

You can also run `fox [SUBCOMMAND] -h` to view a help message.

#### Build
> [!IMPORTANT]
> Building must be performed before local installing or publishing custom extensions. Additionally, building is a prerequisite for `lint.py` (eslint) and other Foxglove developer tools.

To build, run:
```
fox build [OPTIONS]
```
- `--skip-ci`: Use existing node_modules instead of clean installing external dependencies.

By default, `fox build` does several things:
1. Install external dependencies to `node_modules` using `npm ci`.
2. Patch external dependencies using `patch-package`.
3. Build local dependencies in `foxglove/shared/`.
4. Build per-extension dependencies by running `npm run build`.

> [!TIP]
> After building once, you only need to rebuild after changes are made to the extension dependencies (e.g., `package.json`, anything in `foxglove/shared/`, etc.).

#### Install
To perform a local install, run:
```
fox install [extensions ...]
```
- `extensions`: A list of extensions to install. If no extensions are given, all extensions are installed.

This will execute the command `npm run local-install` for each extension specified.

> [!NOTE]
> Extensions are installed at `~/.foxglove-studio/extensions/` and will have the prefix `dukerobotics.ros-jazzy-`.

#### Watch
To watch an extension for changes, run:
```bash
fox watch extension
```
This will automatically execute `npm run local-install` upon `.ts` or `.tsx` file changes in the `src` directory.

#### Publish
> [!IMPORTANT]
> To publish extensions, you must set up the `.foxgloverc` file. See [SETUP.md](../SETUP.md#set-up-the-foxgloverc-file) for more details.

When you are ready to publish custom extensions to your organization, run:
```
fox publish [extensions ...]
```
- `extensions`: A list of extensions to install. If no extensions are given, all extensions are installed.
- `-v, --version`: Version to publish extensions under. If no version is given, the short (length 7) HEAD commit hash is used. A version is required if the `robosub-ros2` git reposititory is dirty.

#### Clean
To clean up the Foxglove monorepo, run:
```
fox clean
```
This launches an interactive session where you can remove files generated during `fox build`.

#### Uninstall
To uninstall all Duke Robotics extensions, run:
```bash
fox uninstall
```

This removes all files generated during `fox install`.

#### Doctor
To troubleshoot the Foxglove development environment, run:
```
fox doctor
```
This command will exit with a non-zero status if any potential problems are found.

## Using Foxglove Studio
> [!NOTE]
> When testing local extensions, the desktop version of Foxglove Studio must be used.

1. Run the following command to start the Foxglove bridge:
    ```bash
    fg-ws
    ```
    - This is an alias that starts the Foxglove bridge, which enables Foxglove Studio to connect to the ROS 2 network.
    - The bridge opens a WebSocket on port `28765`. This port is mapped to port `28765` on the host machine, so you can connect to the WebSocket from your host machine.
2. Open Foxglove Studio and connect to the WebSocket at `ws://IP_ADDRESS:28765`.
    - Replace `IP_ADDRESS` with the IP address of the host machine. If you are running the Docker container locally, you can use `localhost` as the IP address.

## Linting & Formatting
We use [eslint](https://eslint.org/) for linting and [prettier](https://prettier.io/) for formatting. To lint the Foxglove monorepo, run the following command from the `robosub-ros2` root:
```bash
./lint.py -p foxglove
```

See [SCRIPTS.md](../SCRIPTS.md#lintpy) for more information.

## Monorepo Structure
### Extensions
- [`call-service-panel`](extensions/call-service-panel/README.md) - Example panel to call services
- [`publish-topic-panel`](extensions/publish-topic-panel/README.md) - Example panel to publish topics
- [`subscribe-topic-panel`](extensions/subscribe-topic-panel/README.md) - Example panel to subscribe to topics
- [`toggle-controls-panel`](extensions/toggle-controls-panel/README.md) - Panel to toggle controls on/off
- [`system-status-panel`](extensions/system-status-panel/README.md) - Panel that displays system usage of the onboard computer
- [`sensors-status-panel`](extensions/sensors-status-panel/README.md) - Panel that displays the connected/disconnected status of sensors
- [`thruster-allocs-panel`](extensions/thruster-allocs-panel/README.md) - Panel that displays the current thruster allocs and publishes desired thruster allocs
- [`toggle-joystick-panel`](extensions/toggle-joystick-panel/README.md) - Panel to toggle joystick control on/off, as well as publish transformed joystick inputs as a desired power
- [`pid-panel`](extensions/pid-panel/README.md) - Panel to read/set PID gains

### Local Dependencies
Local dependencies are located in the `shared/` directory.
- [`defs`](shared/defs/README.md) - Exports Foxglove datatype maps and TypeScript interfaces/enums for both ROS 2 and Duke Robotics message definitions
- [`theme`](shared/theme/README.md) Exports the Duke Robotics MUI Theme

### Patches
Patches to external node modules are located in the `patches/` directory.
Running `fox build` will automatically apply these patches.
- [`create-foxglove-extension+1.0.4.patch`](patches/create-foxglove-extension+1.0.4.patch)
  - No longer require `README.md` or `CHANGELOG.md` when installing an extension
  - Before installing an extension, only remove `dist/extension.js` (instead of cleaning the entire `dist` directory)

### Monorepo Root Files
- [`eslint.config.mjs`](eslint.config.mjs) - Configuration file for ESLint
- [`.eslintignore`](.eslintignore) - Ignore file for ESLint
- [`.npmrc`](.npmrc) - Configuration file for npm
- [`.prettierrc.yaml`](.prettierrc.yaml) - Configuration file for Prettier
- [`tsconfig.json`](tsconfig.json) Configuration file for TypeScript
- [`package.json`](package.json) - Metadata file for the Foxglove monorepo
- [`package-lock.json`](package-lock.json) - The npm dependency tree of the Foxglove monorepo, generated using `npm i`

## Contributing
### Adding a New Extension
Copy an existing Duke Robotics example extension (`call-service-panel`, `publish-topic-panel`, or `subscribe-topic-panel`) as a starting point. This ensures that all of our extensions have the same code structure and use the same core set of dependencies.

### Adding a New Local Dependency
All local dependencies must expose an `npm run build` command in `package.json`. During build, `foxglove.py` will compile each local dependency to `node_modules` where they can be consumed by an extension.

## Additional Documentation
See the [Foxglove documentation](https://docs.foxglove.dev/docs/visualization/extensions/introduction) for more details on extension development.
