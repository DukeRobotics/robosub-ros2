#!/usr/bin/env python3
"""
A CLI to automatically install/uninstall Foxglove extensions.

To install a specific extension, use the -e flag:
python foxglove.py install -e <extension-1> <extension-2> ...
By default, the `-e` flag without arguments will install all extensions.

For more information, use the -h flag:
python foxglove.py -h
python foxglove.py install -h
python foxglove.py uninstall -h
"""

import argparse
import functools
import json
import pathlib
import platform
import shutil
import subprocess
from collections.abc import Sequence

import git

ORGANIZATION = 'dukerobotics'

if (SYSTEM := platform.system()) not in ('Linux', 'Darwin', 'Windows'):
    msg = f'Unsupported platform: {SYSTEM}'
    raise SystemExit(msg)

EXTENSION_INSTALL_PATH = pathlib.Path.home() / '.foxglove-studio/extensions/'

FOXGLOVE_PATH = pathlib.Path(__file__).parent.resolve()
EXTENSION_PATHS = [d for d in (FOXGLOVE_PATH / 'extensions').iterdir() if d.is_dir()]


def run_at_path(command: str | Sequence[str], directory: pathlib.Path, system: str = SYSTEM,
                windows_append_cmd: bool = True) -> None:
    """
    Run a command at a given path.

    Args:
        command: Command to run.
        directory: Path to run command at.
        system: Can be "Linux", "Darwin", or "Windows". Defaults to platform.system().
        windows_append_cmd: If True, append ".cmd" to command on Windows systems.

    Raises:
        ValueError: If command empty.
        subprocess.CalledProcessError: If command returns non-zero exit code.
    """
    if not command:
        msg = 'Command must not be empty'
        raise ValueError(msg)

    args = command.split(' ') if isinstance(command, str) else list(command)

    if system == 'Windows' and windows_append_cmd:
        args[0] += '.cmd'

    print(f'{directory.name}: {command}')
    subprocess.run(args, cwd=directory, check=True)  # noqa: S603


def check_npm() -> None:
    """
    Check if npm is installed.

    Raises:
        SystemExit: If npm not found.
    """
    try:
        run_at_path('npm -v', FOXGLOVE_PATH)
    except (FileNotFoundError, subprocess.CalledProcessError) as e:
        msg = 'npm not found. Install npm and try again.'
        raise SystemExit(msg) from e


def check_foxglove_cli() -> None:
    """
    Check if the Foxglove CLI is installed.

    Raises:
        SystemExit: If the Foxglove CLI is not found.
    """
    try:
        run_at_path('foxglove version', FOXGLOVE_PATH)
    except (FileNotFoundError, subprocess.CalledProcessError) as e:
        msg = 'The Foxglove CLI was not found. Install the Foxglove CLI and try again.'
        raise SystemExit(msg) from e


def build_deps(skip_ci: bool = False, extension_paths: Sequence[pathlib.Path] = EXTENSION_PATHS) -> None:
    """
    Build all necessary dependencies for Foxglove.

    Args:
        skip_ci: If True, use existing node_modules instead of clean installing external dependencies.
        extension_paths: Sequence of extension paths to build.
    """
    run = functools.partial(run_at_path, directory=FOXGLOVE_PATH)

    # Create necessary paths (either directories or empty files) so that npm ci can symlink them to node_modules
    # Files must have a suffix, otherwise they are treated as directories
    paths = [
        FOXGLOVE_PATH / 'shared/ros-typescript-generator/build/main/cli/cli.js',
    ]
    for path in paths:
        if path.suffix:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.touch()
        else:
            path.mkdir(parents=True, exist_ok=True)

    # Install external dependencies and symlink local dependencies
    if not skip_ci:
        run('npm ci')

        # Patch external dependencies
        run('npx patch-package --patch-dir patches')

    # Compile local shared dependencies
    dependencies = ['ros-typescript-generator', 'defs', 'theme']  # Specify build order
    for dep in dependencies:
        run_at_path('npm run build --if-present', FOXGLOVE_PATH / 'shared' / dep)

    # Compile local nonshared dependencies
    for extension in extension_paths:
        run_at_path('npm run build-deps --if-present', extension)


def install_extensions(extension_paths: Sequence[pathlib.Path]) -> None:
    """
    Install custom Foxglove extensions.

    Args:
        extension_paths: Sequence of extension paths to install.
    """
    successes = 0
    for extension in extension_paths:
        if not (extension / 'package.json').is_file():
            print(f'{extension.name}: skipped (no package.json)')
            continue

        run_at_path('npm run local-install', extension)

        print(f'{extension.name}: installed')

        successes += 1

    print(f'Successfully installed {successes} extension(s)\n')


def _update_extension_version(extension: pathlib.Path, version: str) -> None:
    """
    Update the package.json version field for an extension.

    Args:
        extension: Path to extension.
        version: Version to write to package.json. Must conform to the semver specification.
    """
    with (extension / 'package.json').open() as file:
        package = json.load(file)
    package['version'] = version
    with (extension / 'package.json').open('w') as file:
        json.dump(package, file, indent=2)


def publish_extensions(extension_paths: Sequence[pathlib.Path], version: str | None = None) -> None:
    """
    Publish custom Foxglove extensions.

    Args:
        extension_paths: Sequence of extension paths to publish.
        version: Version to publish extensions under. If None, the short HEAD commit hash is used.

    Raises:
        SystemExit: If the Foxglove directory is dirty and no version is given.
    """
    repo = git.Repo(path=FOXGLOVE_PATH.parent)
    is_dirty = repo.is_dirty(untracked_files=True, path=FOXGLOVE_PATH)
    if is_dirty and version is None:
        msg = 'The foxglove directory is dirty! Commit changes before publishing.'
        raise SystemExit(msg)
    if version is None:
        version = repo.head.object.hexsha[:7]

    successes = 0
    for extension in extension_paths:
        if not (extension / 'package.json').is_file():
            print(f'{extension.name}: skipped (no package.json)')
            continue

        # Update package.json version
        _update_extension_version(extension, version)

        # Build extension package
        run_at_path('npm run package', extension)

        # Publish extension package
        package_name = f'{ORGANIZATION}.{extension.name}-{version}.foxe'
        try:
            run_at_path(f'foxglove extensions publish {package_name}', extension)
        finally:
            # Clean up
            run_at_path(f'rm {package_name}', extension) # Remove extension package
            _update_extension_version(extension, '0.0.0') # Revert package.json version

        print(f'{extension.name}: published')

        successes += 1

    print(f'Successfully published {successes} extension(s)\n')


def uninstall_extensions(install_path: pathlib.Path = EXTENSION_INSTALL_PATH) -> None:
    """
    Uninstall all Duke Robotics extensions from Foxglove.

    Duke Robotics extensions are identified with the prefix 'dukerobotics'.

    Args:
        install_path: Path where extensions are installed.
    """
    extensions = [d for d in install_path.iterdir() if d.name.startswith(ORGANIZATION)]
    for extension in extensions:
        shutil.rmtree(extension)
        print(f'{extension.name}: uninstalled')

    print(f'Successfully uninstalled {len(extensions)} extension(s)\n')


def extension_package(name: str, extension_paths: Sequence[pathlib.Path] = EXTENSION_PATHS) -> pathlib.Path:
    """
    Type for argparse that checks if a given extension name is valid.

    Args:
        name: Name of extension to check.
        extension_paths: Sequence of extension paths to check against.

    Raises:
        argparse.ArgumentTypeError: If name is not a valid extension name.

    Returns:
        pathlib.Path: Full path to extension.
    """
    for extension in extension_paths:
        if name == extension.name:
            return extension

    msg = f'{name} is not a valid extension name'
    raise argparse.ArgumentTypeError(msg)


def clean_foxglove() -> None:
    """Clean up the foxglove monorepo."""
    run_at_path('git clean -fdx', FOXGLOVE_PATH, windows_append_cmd=False)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Install/Uninstall Foxglove extensions.')
    subparsers = parser.add_subparsers(dest='action')

    install_parser = subparsers.add_parser(
        'install',
        aliases=['i'],
        help='Install Foxglove extensions. By default, all extensions are installed.',
    )
    install_parser.add_argument(
        'extensions',
        nargs='*',
        type=extension_package,
        help='Install extension(s) by name. If no name(s) are given, all extensions are installed.',
    )
    install_parser.add_argument(
        '--skip-ci',
        action='store_true',
        help='Use existing node_modules instead of clean installing external dependencies.',
    )
    install_parser.add_argument(
        '--skip-build',
        action='store_true',
        help='Skip preliminary build steps.',
    )

    uninstall_parser = subparsers.add_parser(
        'uninstall',
        aliases=['u'],
        help='Uninstall Foxglove extensions. By default, all extensions are uninstalled.',
    )

    build_parser = subparsers.add_parser(
        'build',
        aliases=['b'],
        help='Build all necessary dependencies for Foxglove. This is automatically run when installing extensions.',
    )
    build_parser.add_argument(
        '--skip-ci',
        action='store_true',
        help='Use existing node_modules instead of clean installing external dependencies.',
    )

    clean_parser = subparsers.add_parser(
        'clean',
        aliases=['c'],
        help='Clean up the foxglove monorepo.',
    )

    publish_parser = subparsers.add_parser(
        'publish',
        aliases=['p'],
        help='Publish Foxglove extensions. By default, all extensions are published.',
    )
    publish_parser.add_argument(
        'extensions',
        nargs='*',
        type=extension_package,
        help='Specify extension(s) to publish. If no name(s) are given, all extensions are published.',
    )
    publish_parser.add_argument(
        '-v', '--version',
        action='store',
        nargs='?',
        help='Version to publish extensions under. If no version is given, the short HEAD commit hash is used.',
    )

    args = parser.parse_args()

    if args.action in ('install', 'i'):
        # Without flags, install everything
        if args.extensions is None:
            args.extensions = EXTENSION_PATHS
        # If only the -e flag is set without additional arguments, install all extensions
        if args.extensions == []:
            args.extensions = EXTENSION_PATHS

        if args.extensions is not None:
            check_npm()
            if not args.skip_build:
                build_deps(skip_ci=args.skip_ci, extension_paths=args.extensions)
            install_extensions(args.extensions)

    elif args.action in ('publish', 'p'):
        # Without flags, publish all extensions
        if args.extensions is None or args.extensions == []:
            args.extensions = EXTENSION_PATHS

        check_npm()
        check_foxglove_cli()
        publish_extensions(args.extensions, args.version)

    elif args.action in ('uninstall', 'u'):
        uninstall_extensions()

    elif args.action in ('build', 'b'):
        check_npm()
        build_deps(skip_ci=args.skip_ci)

    elif args.action in ('clean', 'c'):
        clean_foxglove()
