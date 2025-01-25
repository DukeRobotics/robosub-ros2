#!/usr/bin/env python3
"""A CLI that manages custom extensions for Foxglove Studio."""

import argparse
import functools
import json
import pathlib
import shutil
import subprocess
from collections.abc import Sequence

import git

ORGANIZATION = 'dukerobotics'
ROS_DISTRO = 'ros-jazzy'

EXTENSION_INSTALL_PATH = pathlib.Path.home() / '.foxglove-studio/extensions/'

FOXGLOVE_PATH = pathlib.Path(__file__).parent.resolve()
ROOT_PATH = FOXGLOVE_PATH.parent.resolve()
EXTENSION_PATHS = [d for d in (FOXGLOVE_PATH / 'extensions').iterdir() if d.is_dir()]

STATUS_EMOJI = {
    True: '✅',
    False: '❌',
}


def run_at_path(command: str, directory: pathlib.Path) -> None:
    """
    Run a command at a given path.

    Args:
        command: Command to run.
        directory: Path to run command at.

    Raises:
        ValueError: If command empty.
        subprocess.CalledProcessError: If command returns non-zero exit code.
    """
    if not command:
        msg = 'Command must not be empty'
        raise ValueError(msg)

    print(f'{directory.resolve()}# {command}')

    subprocess.run(command, cwd=directory, check=True, shell=True)  # noqa: S602


def doctor() -> None:
    """
    Check if the Foxglove development environment is set up correctly.

    Raises:
        SystemExit: If any potential problems are found.
    """
    num_success = 0

    commands = {
        'npm': 'npm -v',
        'foxglove cli': 'foxglove version',
        'foxglove cli authentication': 'foxglove auth info',
    }

    for title, command in commands.items():
        try:
            run_at_path(command, FOXGLOVE_PATH)
            print(f'{STATUS_EMOJI[True]} {title} found')
            num_success += 1
        except (FileNotFoundError, subprocess.CalledProcessError):
            print(f'{STATUS_EMOJI[False]} {title} missing!')
        print()

    print(f'{num_success}/{len(commands)} checks passed')

    if num_success != len(commands):
        raise SystemExit(1)


def build(skip_ci: bool = False, extension_paths: Sequence[pathlib.Path] = EXTENSION_PATHS) -> None:
    """
    Build all necessary dependencies for Foxglove.

    Args:
        skip_ci: If True, use existing node_modules instead of clean installing external dependencies.
        extension_paths: Sequence of extension paths to build.
    """
    run = functools.partial(run_at_path, directory=FOXGLOVE_PATH)

    # Install external dependencies and symlink local dependencies
    if not skip_ci:
        print('Installing external dependencies. This may take some time...')
        run('npm ci')

        # Patch external dependencies
        run('npx patch-package --patch-dir patches')

    # Compile local shared dependencies
    dependencies = ['defs', 'theme']  # Specify build order
    for dep in dependencies:
        run_at_path('npm run build --if-present', FOXGLOVE_PATH / 'shared' / dep)

    # Compile local nonshared dependencies
    for extension in extension_paths:
        run_at_path('npm run build --if-present', extension)


def install(extension_paths: Sequence[pathlib.Path]) -> None:
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

    print(f'Successfully installed {successes} extension(s)')


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


def publish(extension_paths: Sequence[pathlib.Path], version: str | None = None) -> None:
    """
    Publish custom Foxglove extensions.

    Args:
        extension_paths: Sequence of extension paths to publish.
        version: Version to publish extensions under. If None, the short HEAD commit hash is used.

    Raises:
        SystemExit: If the Foxglove directory is dirty and no version is given.
    """
    repo = git.Repo(path=FOXGLOVE_PATH.parent)
    is_dirty = repo.is_dirty(untracked_files=True)
    if is_dirty and version is None:
        msg = 'Commit changes before publishing or use the --version flag.'
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
        package_name = f'{ORGANIZATION}.{ROS_DISTRO}-{extension.name}-{version}.foxe'
        try:
            run_at_path(f'foxglove extensions publish {package_name}', extension)
        finally:
            # Clean up
            run_at_path(f'rm {package_name}', extension) # Remove extension package
            _update_extension_version(extension, '0.0.0') # Revert package.json version

        print(f'{extension.name}: published')

        successes += 1

    print(f'Successfully published {successes} extension(s)')


def uninstall(install_path: pathlib.Path = EXTENSION_INSTALL_PATH) -> None:
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

    print(f'Successfully uninstalled {len(extensions)} extension(s)')

def watch(extension: pathlib.Path) -> None:
    """
    Watch a Foxglove extension for changes and rebuild on save.

    Args:
        extension: Extension to watch for changes.
    """
    command = "npx nodemon --watch './src/**' -e ts,tsx --exec 'npm run local-install'"
    run_at_path(command, extension)


def extension_package(name: str) -> pathlib.Path:
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
    extension = (FOXGLOVE_PATH / 'extensions' / name)
    if extension.is_dir():
        return extension

    msg = f'{name} is not a valid extension name'
    raise argparse.ArgumentTypeError(msg)


def clean() -> None:
    """Clean up the foxglove monorepo."""
    run_at_path('git clean -ix foxglove', ROOT_PATH)


def main() -> None:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(description='Manage custom extensions for Foxglove Studio.')
    subparsers = parser.add_subparsers(dest='action')

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

    install_parser = subparsers.add_parser(
        'install',
        aliases=['i'],
        help='Install Foxglove extensions. By default, all extensions are installed.',
    )
    install_parser.add_argument(
        'extensions',
        nargs='*',
        type=extension_package,
        help='Install extension(s) by name.',
    )

    watch_parser = subparsers.add_parser(
        'watch',
        aliases=['w'],
        help='Watch a Foxglove extension for changes and rebuild on save.',
    )
    watch_parser.add_argument(
        'extensions',
        metavar='extension',
        nargs=1,
        type=extension_package,
        help='Extension to watch for changes.',
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
        help='Specify extension(s) to publish. If no names are given, all extensions are published.',
    )
    publish_parser.add_argument(
        '-v', '--version',
        action='store',
        nargs='?',
        help='Version to publish extensions under. If no version is given, the short HEAD commit hash is used.',
    )

    subparsers.add_parser(
        'clean',
        aliases=['c'],
        help='Clean up the Foxglove monorepo.',
    )

    subparsers.add_parser(
        'uninstall',
        aliases=['u'],
        help='Uninstall all Foxglove extensions.',
    )

    subparsers.add_parser(
        'doctor',
        aliases=['d'],
        help='Troubleshoot the Foxglove development environment.',
    )


    args = parser.parse_args()
    if args.action in {'build', 'b'}:
        build(skip_ci=args.skip_ci)
    elif args.action in {'install', 'i'}:
        install(args.extensions or EXTENSION_PATHS)
    elif args.action in {'watch', 'w'}:
        watch(args.extensions[0])
    elif args.action in {'publish', 'p'}:
        publish(args.extensions or EXTENSION_PATHS, args.version)
    elif args.action in {'clean', 'c'}:
        clean()
    elif args.action in {'uninstall', 'u'}:
        uninstall()
    elif args.action in {'doctor', 'd'}:
        doctor()
    else:
        parser.print_help()

if __name__ == '__main__':
    main()
