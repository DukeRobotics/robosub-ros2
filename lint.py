#!/usr/bin/env python3
"""
This script lints Python, C++, and Bash files in a specified directory or file.

It uses ruff for Python, clang-format for C++, and shellcheck for Bash.
"""

import argparse
import os
import subprocess
from collections.abc import Generator
from dataclasses import dataclass
from enum import Enum
from pathlib import Path

from git import Repo


@dataclass
class LintLanguageProperties:
    """
    Dataclass to store properties of a programming language.

    Attributes:
        name (str): The name of the programming language.
        file_extensions (list[str]): A list of file extensions associated with the programming language.
        lint_command (str): The command to run to lint files of the programming language. The command should contain a
            placeholder `{path}` for the file path to lint.
        autofix_command (str | None): The command to run to autofix linting errors in files of the programming
            language. The command should contain a placeholder `{path}` for the file path to lint. If None, autofixing
            is not supported for this language.
    """
    name: str
    file_extensions: list[str]
    lint_command: str
    autofix_command: str | None

class LintLanguage(Enum):
    """Enum to specify the programming language for linting."""
    PYTHON = LintLanguageProperties(
        name='python',
        file_extensions=['.py'],
        lint_command='/root/dev/venv/bin/python3 -m ruff check -q {path}',
        autofix_command='/root/dev/venv/bin/python3 -m ruff check --fix -q {path}',
    )
    CPP = LintLanguageProperties(
        name='cpp',
        file_extensions=['.cpp', '.h', '.c', '.hpp'],
        lint_command='clang-format -style=file --Werror --dry-run {path}',
        autofix_command='clang-format -style=file -i {path}',
    )
    BASH = LintLanguageProperties(
        name='bash',
        file_extensions=['.sh'],
        lint_command='shellcheck {path}',
        autofix_command='shellcheck -f diff {path} | git apply --allow-empty && shellcheck {path}',
    )

# Map from LintLanguage name to LintLanguage
STR_TO_LINT_LANGUAGE = {lang.value.name: lang for lang in LintLanguage}

# List of lint language names
LINT_LANGUAGE_NAMES = sorted([lang.value.name for lang in LintLanguage])

# Map from file extension to LintLanguage
FILE_EXTENSIONS_TO_LINT_LANGUAGES = {ext: lang for lang in LintLanguage for ext in lang.value.file_extensions}

# Maximum length of a language name
MAX_LANGUAGE_LENGTH = max(len(lang.value.name) for lang in LintLanguage)

# Set of languages with autofix commands
LINT_LANGUAGE_WITH_AUTOFIX_NAMES = sorted([lang.value.name for lang in LintLanguage \
                                           if lang.value.autofix_command is not None])

# Map from lint status to emoji
STATUS_EMOJI = {
    True: '✅',
    False: '❌',
}

# Path to the repository and default path to lint
REPO_PATH = Path('/root/dev/robosub-ros2')

@dataclass
class LintLanguageStats:
    """
    Dataclass to store linting statistics for a specific language.

    Attributes:
        total (int): The total number of files linted for the language.
        success (int): The number of files successfully linted for the language
    """
    total: int = 0
    success: int = 0

class LintOutputType(Enum):
    """
    Enum to specify the output type of the linting command.

    Attributes:
        CAPTURE (int): Capture the output of the linting command and print it through this script.
        TERMINAL (int): Print the output of the linting command directly to the terminal.
        QUIET (int): Suppress the output of the linting command.
    """
    CAPTURE = 0
    TERMINAL = 1
    QUIET = 2

# Map from terminal to LintOutputType
STR_TO_LINT_OUTPUT_TYPE = {
    'capture': LintOutputType.CAPTURE,
    'terminal': LintOutputType.TERMINAL,
    'quiet': LintOutputType.QUIET,
}

# Map from LintOutputType to subprocess.PIPE, None, or subprocess.DEVNULL
LINT_OUTPUT_TYPE_TO_SUBPROCESS = {
    LintOutputType.CAPTURE: subprocess.PIPE,
    LintOutputType.TERMINAL: None,
    LintOutputType.QUIET: subprocess.DEVNULL,
}

def lint_file(file_path: Path, language: LintLanguage, autofix: bool, print_success: bool,
              output_type: LintOutputType) -> bool:
    """
    Lint a single file using the appropriate linter for the specified language.

    Args:
        file_path (Path): The path of the file to lint.
        language (LintLanguage): The programming language of the file.
        autofix (bool): If True, attempt to autofix linting errors. This modifies the file in place. This is only
            supported for languages with autofix commands.
        print_success (bool): If True, print a success message when linting is successful.
        output_type (LintOutputType): How to handle the output of the linting command.
    """
    base_command = language.value.autofix_command if autofix and language.value.autofix_command else \
        language.value.lint_command
    file_path = file_path.relative_to(REPO_PATH)
    command = base_command.format(path=file_path)

    # Pad the language name to align the output for all languages
    padded_language = f'{language.value.name}:'.ljust(MAX_LANGUAGE_LENGTH + 1)
    subprocess_output_type = LINT_OUTPUT_TYPE_TO_SUBPROCESS[output_type]

    process = subprocess.Popen(command, stdout=subprocess_output_type, stderr=subprocess_output_type, shell=True, \
                               cwd=REPO_PATH) # noqa: S602
    stdout, _ = process.communicate()

    # If the process returns 0, the linting was successful
    if process.returncode == 0:
        if print_success:
            print(f'{STATUS_EMOJI[True]} {padded_language} {file_path}')
        return True

    # The linting failed. Print the output if it was captured.
    if output_type == LintOutputType.CAPTURE and stdout:
        indented_output = '\n'.join('    ' + line for line in stdout.decode().splitlines())
        print(indented_output)
    print(f'{STATUS_EMOJI[False]} {padded_language} {file_path}')
    return False


def traverse_directory(target_path: Path, check_if_git_ignored: bool) -> Generator[Path, None, None]:
    """
    Traverse files in the specified directory.

    Will not include files that are part of a `.git` directory. Can optionally avoid files ignored by Git.

    Args:
        target_path (Path): The directory to traverse. Must be within a Git repository.
        check_if_git_ignored (bool): If True, don't yield files ignored by git. Else, yield all files.

    Yields:
        Path: The absolute path of each file.
    """
    # Initialize the Git repository
    if check_if_git_ignored:
        repo = Repo(target_path, search_parent_directories=True)

    # Ensure the target path is absolute
    abs_target_path = target_path.resolve()

    # Walk through the directory
    for root, dirs, files in os.walk(abs_target_path):
        root_path = Path(root).resolve()

        # Skip this directory and its children if .git folder is in the path
        if '.git' in root_path.parts:
            print(f'Skipping {root_path}...')
            dirs.clear()
            continue

        # Yield files in the current directory that are not git-ignored (if enabled)
        for file_name in files:
            file_path = root_path / file_name
            if not check_if_git_ignored or not repo.ignored(file_path):
                yield file_path

        # Filter directories to exclude git-ignored ones (if enabled) and directories that are within a .git folder
        filtered_dirs = []
        for d in dirs:
            dir_path = root_path / d
            if check_if_git_ignored and repo.ignored(dir_path):
                continue
            if '.git' in dir_path.parts:
                continue
            filtered_dirs.append(d)

        # Update directories that will be traversed
        dirs[:] = filtered_dirs


def lint_files(target_path: Path, languages: list[LintLanguage], autofix: bool, print_success: bool,
               output_type: LintOutputType, no_git_tree: bool) -> tuple[bool, dict[LintLanguage, LintLanguageStats]]:
    """
    Lint files in the specified directory or file.

    Args:
        target_path (Path): The directory or file to lint.
        languages (list[LintLanguage]): The programming languages to lint.
        autofix (bool): If True, attempt to autofix linting errors. This modifies the files in place. This is only
            supported for languages with autofix commands.
        print_success (bool): If True, print a success message when linting is successful. If False, only print failed
            linting messages.
        output_type (LintOutputType): How to handle the output of the linting commands.
        no_git_tree (bool): If True, do not check if files are ignored by git and lint all files in the specified
            directory. Otherwise, only lint files that are not ignored by git.

    Returns:
        tuple[bool, dict[LintLanguage, LintLanguageStats]]: A tuple containing a boolean indicating if all files were
            successfully linted and a dictionary of the linting statistics for each language.
    """
    language_stats = {lang: LintLanguageStats() for lang in languages}
    all_success = True
    prev_success = True

    # Traverse the directory and lint each file
    for file_path in traverse_directory(target_path, not no_git_tree):

        # If the file extension is associated with a language to lint, lint the file
        detected_language = FILE_EXTENSIONS_TO_LINT_LANGUAGES.get(file_path.suffix)
        if detected_language in languages:
            language_stats[detected_language].total += 1

            # Print a newline between files if the previous file failed for readability
            if not prev_success and output_type != LintOutputType.QUIET:
                print()

            success = lint_file(file_path, detected_language, autofix, print_success, output_type)
            if success:
                language_stats[detected_language].success += 1

            all_success = all_success and success
            prev_success = success

    return all_success, language_stats


def print_summary(language_stats: dict[LintLanguage, LintLanguageStats]) -> None:
    """
    Print a summary of the linting results.

    Prints the number of files successfully linted and the total number of files for each language that was linted and
        for all languages combined.

    Args:
        language_stats (dict[LintLanguage, LintLanguageStats]): Linting statistics for each language.
    """
    cumulative_success_count = sum(stats.success for stats in language_stats.values())
    cumulative_total_count = sum(stats.total for stats in language_stats.values())
    cumulative_emoji = STATUS_EMOJI[cumulative_success_count == cumulative_total_count]

    print()
    print(f'{cumulative_emoji} Linting Summary: {cumulative_success_count}/{cumulative_total_count}')
    for lang, stats in language_stats.items():
        lang_emoji = STATUS_EMOJI[stats.success == stats.total]
        print(f'  {lang_emoji} {lang.value.name.capitalize()}: {stats.success}/{stats.total}')
    print()


def main() -> None:
    """Parse command-line arguments and initiate the linting process."""
    parser = argparse.ArgumentParser(description='Lint files.')
    parser.add_argument('-p', '--path', nargs='?', default=REPO_PATH,
                        help='Path to the directory or file to lint. Defaults to the entire repository.')
    parser.add_argument('-l', '--languages', choices=LINT_LANGUAGE_NAMES, nargs='+',
                        default=LINT_LANGUAGE_NAMES,
                        help=f'Language(s) to lint ({", ".join(LINT_LANGUAGE_NAMES)}). '
                              'Defaults to linting all supported languages.')
    parser.add_argument('-f', '--fix', action='store_true',
                        help='Attempt to autofix linting errors by modifying files in place. '
                             'Autofix is available for these languages: '
                             f'{", ".join(LINT_LANGUAGE_WITH_AUTOFIX_NAMES)}.')
    parser.add_argument('--print-success', action='store_true',
                        help='Print the names of files that were successfully linted. This is automatically enabled if '
                             '--path is a file.')
    parser.add_argument('-o', '--output-type', choices=STR_TO_LINT_OUTPUT_TYPE.keys(), default='terminal',
                        help=('How to handle the outputs of the linting commands. '
                              '"capture" captures and prints the output through this script (useful for CI/CD). '
                              '"terminal" prints the output directly to the terminal. '
                              '"quiet" suppresses the output. '
                              'Default is "terminal".'))
    parser.add_argument('-s', '--sort', action='store_true',
                        help='Sort the output by language.')
    parser.add_argument('--github-action', action='store_true',
                        help='Use GitHub Actions workflow commands in the output.')
    parser.add_argument('--no-git-tree', action='store_true',
                        help='Do not check if files are ignored by git. Instead, lint all files in the '
                             'specified directory, including git-ignored files. This is useful when this script is run '
                             'in a CI/CD environment that does not have the git tree available.')
    args = parser.parse_args()

    target_path = Path(args.path).resolve()

    if not target_path.exists():
        error_msg = f'The specified path "{target_path}" does not exist.'
        raise FileNotFoundError(error_msg)

    # Ensure target_path is in the repository
    if not target_path.is_relative_to(REPO_PATH):
        error_msg = f'The specified path "{target_path}" must be within the repository located at "{REPO_PATH}".'
        raise ValueError(error_msg)

    # Convert string arguments to their respective enum types
    output_type = STR_TO_LINT_OUTPUT_TYPE[args.output_type]
    languages = [STR_TO_LINT_LANGUAGE[lang] for lang in args.languages]

    # Whether all files were successfully linted
    all_success = True

    if target_path.is_file():
        # If a specific file is provided, ensure it is part of the specified language(s)
        file_language = FILE_EXTENSIONS_TO_LINT_LANGUAGES.get(target_path.suffix)
        if file_language not in languages:
            error_msg = (f'The specified file\'s extension "{target_path.suffix}" does not match the specified '
                         f'language(s): {", ".join(args.languages)}.')
            raise ValueError(error_msg)

        all_success = lint_file(target_path, file_language, args.fix, True, output_type)

    elif args.sort:
        # Lint one language at a time to group the output by language
        aggregate_language_stats = {}
        for language in languages:

            # Use GitHub Actions workflow commands to group the output by language
            if args.github_action:
                print(f'::group::Lint {language.value.name.capitalize()}')
            else:
                print(f'\nLinting {language.value.name.capitalize()} files...')
            lang_success, language_stats = lint_files(target_path, [language], args.fix, args.print_success,
                                                      output_type, args.no_git_tree)
            if args.github_action:
                print('::endgroup::')

            all_success = lang_success and all_success
            aggregate_language_stats.update(language_stats)

        print_summary(aggregate_language_stats)

    else:
        all_success, language_stats = lint_files(target_path, languages, args.fix, args.print_success, output_type,
                                                 args.no_git_tree)
        print_summary(language_stats)

    if not all_success:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
