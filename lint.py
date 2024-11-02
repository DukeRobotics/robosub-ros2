#!/usr/bin/env python3
"""
This script lints Python, C++, and Bash files in a specified directory or file.

It uses flake8 for Python, clang-format for C++, and shellcheck for Bash.
"""

import argparse
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import List

from git import Repo

LANGUAGES_TO_FILE_EXTENSIONS = {
    'python': ['.py'],
    'cpp': ['.cpp', '.h', '.c', '.hpp'],
    'bash': ['.sh']
}

FILE_EXTENSIONS_TO_LANGUAGES = {ext: lang for lang, exts in LANGUAGES_TO_FILE_EXTENSIONS.items() for ext in exts}

LINT_COMMANDS = {
    'python': ['/root/dev/venv/bin/python3', '-m', 'flake8'],
    'cpp': ['clang-format', '-style=file', '--dry-run'],
    'bash': ['shellcheck']
}

STATUS_EMOJI = {
    True: '✅',
    False: '❌'
}

MAX_LANGUAGE_LENGTH = max(len(language) for language in LANGUAGES_TO_FILE_EXTENSIONS.keys())


@dataclass
class LanguageStats:
    """Dataclass to store linting statistics for a specific language."""

    total: int = 0
    success: int = 0


def lint_file(file_path: Path, language: str, print_success: bool, quiet: bool):
    """
    Lint a single file using the appropriate linter for the specified language.

    Args:
        file_path (Path): The path of the file to lint.
        language (str): The programming language of the file.
        print_success (bool): If True, print a success message when linting is successful.
    """
    command = LINT_COMMANDS[language] + [str(file_path)]
    out = subprocess.DEVNULL if quiet else subprocess.PIPE
    padded_language = f'{language}:'.ljust(MAX_LANGUAGE_LENGTH + 1)

    process = subprocess.Popen(command, stdout=out, stderr=out)
    stdout, stderr = process.communicate()

    if process.returncode == 0:
        if print_success:
            print(f'{STATUS_EMOJI[True]} {padded_language} {file_path}')
        return True
    else:
        if not quiet:
            indented_output = '\n'.join('    ' + line for line in (stdout.decode() if stdout else '').splitlines())
            print(indented_output)
        print(f'{STATUS_EMOJI[False]} {padded_language} {file_path}')
        return False


def print_summary(language_stats: dict[str, LanguageStats]):
    """
    Print a summary of the linting results.

    Args:
        language_stats (dict[str, LanguageStats]): A dictionary containing the linting statistics for each language.
    """
    overall_success = all(stats.success == stats.total for stats in language_stats.values())
    overall_emoji = STATUS_EMOJI[overall_success]
    print()
    print(f"{overall_emoji} Linting Summary (success/total):")
    for lang, stats in language_stats.items():
        lang_emoji = STATUS_EMOJI[stats.success == stats.total]
        print(f"  {lang_emoji} {lang.capitalize()}: {stats.success}/{stats.total}")
    print()


def lint_files(target_path: Path, language: List[str] | None = None, print_success: bool = False, quiet: bool = False,
               no_git_tree: bool = False) -> bool:
    """
    Lint files in the specified directory or file.

    Args:
        target_path (Path): The directory or file to lint.
        language (str, optional): The programming language to lint. If not specified, lint all supported languages.
        print_success (bool): If True, print a success message when linting is successful.
        quiet (bool): If True, suppress output from the linting commands except for success or issue messages.
        no_git_tree (bool): If True, do not use the git tree to traverse only files tracked by git. Instead, lint all
            files in the specified directory.
    """
    repo = Repo(target_path, search_parent_directories=True)
    languages = set(language) if language else LANGUAGES_TO_FILE_EXTENSIONS.keys()
    language_stats = {lang: LanguageStats() for lang in languages}
    all_success = True
    prev_success = True

    def process_file(file_path: Path):
        nonlocal prev_success, all_success

        detected_language = FILE_EXTENSIONS_TO_LANGUAGES.get(file_path.suffix)
        if detected_language in languages:
            language_stats[detected_language].total += 1

            if not prev_success and not quiet:
                print()

            success = lint_file(file_path, detected_language, print_success, quiet)
            if success:
                language_stats[detected_language].success += 1

            all_success = all_success and success
            prev_success = success

    if no_git_tree:
        for file_path in target_path.rglob('*'):
            if file_path.is_file() and target_path in file_path.parents:
                process_file(file_path)
    else:
        for item in repo.tree().traverse():
            file_path = Path(item.path).resolve()
            if item.type == 'blob' and file_path.is_file() and target_path in file_path.parents:
                process_file(file_path)

    return all_success, language_stats


def main():
    """Parse command-line arguments and initiate the linting process."""
    default_path = Path('/root/dev/robosub-ros2')

    parser = argparse.ArgumentParser(description='Lint files.')
    parser.add_argument('path', nargs='?', default=default_path,
                        help='Specify the directory or file to lint. Defaults to the robosub-ros2 directory.')
    parser.add_argument('-l', '--language', choices=LANGUAGES_TO_FILE_EXTENSIONS.keys(), nargs='+',
                        help=f'Specify the language(s) to lint ({", ".join(LANGUAGES_TO_FILE_EXTENSIONS.keys())}). '
                             'If not specified, lint all.')
    parser.add_argument('--print-success', action='store_true',
                        help='If specified, print the names of files that were successfully linted.')
    parser.add_argument('-q', '--quiet', action='store_true',
                        help=('If specified, suppress output from the linting commands except for success or issue '
                              'messages.'))
    parser.add_argument('-s', '--sorted', action='store_true',
                        help='If specified, sort the output by language.')
    parser.add_argument('--github-action', action='store_true',
                        help='If specified, use GitHub Actions workflow commands in the output.')
    parser.add_argument('--no-git-tree', action='store_true',
                        help='If specified, do not use the git tree to traverse only files tracked by git. Instead, '
                             'lint all files in the specified directory, including git-ignored and untracked files.')
    args = parser.parse_args()

    target_path = Path(args.path).resolve()

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

        all_success = lint_file(target_path, language, args.print_success, args.quiet)
    else:
        if args.sorted:
            # Lint one language at a time to group the output by language
            aggregate_language_stats = {}
            for language in LANGUAGES_TO_FILE_EXTENSIONS.keys():
                if args.github_action:
                    print(f'::group::Linting {language.capitalize()} ')
                else:
                    print(f'\nLinting {language.capitalize()} files...')
                lang_success, language_stats = lint_files(target_path, [language], args.print_success, args.quiet,
                                                          args.no_git_tree)
                if args.github_action:
                    print('::endgroup::')

                all_success = lang_success and all_success
                aggregate_language_stats.update(language_stats)

            print_summary(aggregate_language_stats)
        else:
            all_success, language_stats = lint_files(target_path, args.language, args.print_success, args.quiet,
                                                     args.no_git_tree)
            print_summary(language_stats)

    if not all_success:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
