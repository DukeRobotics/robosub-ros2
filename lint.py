#!/usr/bin/env python3
"""
This script lints Python, C++, and Bash files in a specified directory or file.

It uses flake8 for Python, clang-format for C++, and shellcheck for Bash.
"""

import argparse
import fnmatch
import subprocess
import sys
from pathlib import Path
from typing import List

FILE_EXTENSIONS = {
    'python': ['.py'],
    'cpp': ['.cpp', '.h', '.c', '.hpp'],
    'bash': ['.sh']
}

LINT_COMMANDS = {
    'python': ['python3', '-m', 'flake8'],
    'cpp': ['clang-format', '-style=file', '--dry-run'],
    'bash': ['shellcheck']
}

had_errors = False


def load_gitignore_patterns():
    """
    Load patterns from the .gitignore file in the current directory.

    Returns:
        list: A list of patterns to ignore.
    """
    gitignore_path = Path(__file__).parent / '.gitignore'
    if not gitignore_path.exists():
        return []
    with gitignore_path.open() as gitignore_file:
        return [line.strip() for line in gitignore_file if line.strip() and not line.startswith('#')]


def is_ignored(file_path: Path, ignore_patterns: List[str]) -> bool:
    """
    Check if a file path matches any of the ignore patterns.

    Args:
        file_path (Path): The path of the file to check.
        ignore_patterns (List[str]): A list of patterns to ignore.

    Returns:
        bool: True if the file path matches any ignore pattern, False otherwise.
    """
    for pattern in ignore_patterns:
        if fnmatch.fnmatch(str(file_path.relative_to(Path.cwd())), pattern):
            return True
    return False


def lint_file(file_path: Path, language: str, print_success: bool):
    """
    Lint a single file using the appropriate linter for the specified language.

    Args:
        file_path (Path): The path of the file to lint.
        language (str): The programming language of the file (python, cpp, bash).
        print_success (bool): If True, print a success message when linting is successful.
    """
    global had_errors
    command = LINT_COMMANDS[language] + [str(file_path)]
    try:
        subprocess.run(command, check=True)
        if print_success:
            print(f'Successfully linted {language} file: {file_path}')
    except subprocess.CalledProcessError:
        print(f'Issues found in {language} file: {file_path}')
        had_errors = True


def matches_extension(file_path: Path, language: str):
    """
    Check if the file path matches the extension for the specified language.

    Args:
        file_path (Path): The path of the file to check.
        language (str): The programming language to check against.

    Returns:
        bool: True if the file path matches the extension for the language, False otherwise.
    """
    return any(file_path.suffix == ext for ext in FILE_EXTENSIONS[language])


def lint_files(target_path: Path, language: str | None = None, print_success: str | None = False):
    """
    Lint files in the specified directory or file.

    Args:
        target_path (Path): The directory or file to lint.
        language (str, optional): The programming language to lint (python, cpp, bash). If not specified, lint all.
        print_success (bool, optional): If True, print a success message when linting is successful.
    """
    ignore_patterns = load_gitignore_patterns()
    # Prepare the extensions and languages to be used in the linting loop
    languages = [language] if language else FILE_EXTENSIONS.keys()

    # Walk through the target directory and its subdirectories
    for file_path in target_path.rglob('*'):
        if file_path.is_file() and not is_ignored(file_path, ignore_patterns):
            for lang in languages:
                if matches_extension(file_path, lang):
                    lint_file(file_path, lang, print_success)
                    break


def main():
    """Parse command-line arguments and initiate the linting process."""
    parser = argparse.ArgumentParser(description='Lint Python, C++, and Bash files.')
    parser.add_argument('language', nargs='?', choices=['python', 'bash', 'cpp'],
                        help='Specify the language to lint (python, bash, cpp). If not specified, lint all.')
    parser.add_argument('path', nargs='?', default=Path.cwd(),
                        help='Specify the directory or file to lint. Defaults to current directory.')
    parser.add_argument('--print-success', action='store_true',
                        help='If specified, print the names of files that were successfully linted.')
    args = parser.parse_args()

    target_path = Path(args.path)

    if not target_path.exists():
        print(f'Error: The specified path "{target_path}" does not exist.')
        sys.exit(1)

    if target_path.is_file():
        # If a specific file is provided, ensure it matches the language if specified
        if args.language:
            if not matches_extension(target_path, args.language):
                print(f'Error: Specified file is not a {args.language} file.')
                sys.exit(1)

        # Lint the specific file
        for lang in FILE_EXTENSIONS.keys():
            if matches_extension(target_path, lang):
                lint_file(target_path, lang, args.print_success)
                break
        else:
            print('Error: Unsupported file type.')
            sys.exit(1)
    else:
        # If a directory is provided, lint accordingly
        lint_files(target_path, args.language, args.print_success)

    if had_errors:
        sys.exit(1)


if __name__ == '__main__':
    main()
