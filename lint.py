#!/usr/bin/env python3
"""
This script lints Python, C++, and Bash files in a specified directory or file.

It uses flake8 for Python, clang-format for C++, and shellcheck for Bash.
"""


import argparse
import subprocess
from pathlib import Path

from git import Repo

LANGUAGES_TO_FILE_EXTENSIONS = {
    'python': ['.py'],
    'cpp': ['.cpp', '.h', '.c', '.hpp'],
    'bash': ['.sh']
}

FILE_EXTENSIONS_TO_LANGUAGES = {ext: lang for lang, exts in LANGUAGES_TO_FILE_EXTENSIONS.items() for ext in exts}

LINT_COMMANDS = {
    'python': ['python3', '-m', 'flake8'],
    'cpp': ['clang-format', '-style=file', '--dry-run'],
    'bash': ['shellcheck']
}


def lint_file(file_path: Path, language: str, print_success: bool):
    """
    Lint a single file using the appropriate linter for the specified language.

    Args:
        file_path (Path): The path of the file to lint.
        language (str): The programming language of the file (python, cpp, bash).
        print_success (bool): If True, print a success message when linting is successful.
    """
    command = LINT_COMMANDS[language] + [str(file_path)]
    try:
        subprocess.run(command, check=True)
        if print_success:
            print(f'Successfully linted {language} file: {file_path}')
        return True
    except subprocess.CalledProcessError:
        print(f'Issues found in {language} file: {file_path}')
        return False


def lint_files(target_path: Path, language: str | None = None, print_success: str | None = False):
    """
    Lint files in the specified directory or file.

    Args:
        target_path (Path): The directory or file to lint.
        language (str, optional): The programming language to lint (python, cpp, bash). If not specified, lint all.
        print_success (bool, optional): If True, print a success message when linting is successful.
    """
    repo = Repo(target_path, search_parent_directories=True)
    all_success = True

    # Prepare the extensions and languages to be used in the linting loop
    languages = [language] if language else LANGUAGES_TO_FILE_EXTENSIONS.keys()

    for item in repo.tree().traverse():
        if item.type == 'blob' and (file_path := Path(item.path)).is_file() and \
                (language := FILE_EXTENSIONS_TO_LANGUAGES.get(file_path.suffix)) in languages:
            success = lint_file(file_path, language, print_success)
            all_success = all_success and success

    return all_success


def main():
    """Parse command-line arguments and initiate the linting process."""
    default_path = Path('/root/dev/robosub-ros2')

    parser = argparse.ArgumentParser(description='Lint Python, C++, and Bash files.')
    parser.add_argument('language', nargs='?', choices=['python', 'bash', 'cpp'],
                        help='Specify the language to lint (python, bash, cpp). If not specified, lint all.')
    parser.add_argument('path', nargs='?', default=default_path,
                        help='Specify the directory or file to lint. Defaults to the robosub-ros2 directory.')
    parser.add_argument('--print-success', action='store_true',
                        help='If specified, print the names of files that were successfully linted.')
    args = parser.parse_args()

    target_path = Path(args.path)

    if not target_path.exists():
        raise FileNotFoundError(f'The specified path "{target_path}" does not exist.')

    # Ensure target_path is the default directory or a subpath of it
    if not target_path.is_relative_to(default_path):
        raise ValueError(f'The specified path "{target_path}" must be within the default script directory '
                         f'"{default_path}".')

    all_success = True

    if target_path.is_file():
        # If a specific file is provided, ensure it matches the language if specified
        if args.language and not (target_path.suffix in LANGUAGES_TO_FILE_EXTENSIONS[args.language]):
            raise ValueError(f'Specified file is not a {args.language} file.')

        language = FILE_EXTENSIONS_TO_LANGUAGES.get(target_path.suffix)
        if not language:
            raise ValueError('Unsupported file type.')

        all_success = lint_file(target_path, language, args.print_success)
    else:
        # If a directory is provided, lint accordingly
        all_success = lint_files(target_path, args.language, args.print_success)

    if not all_success:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
