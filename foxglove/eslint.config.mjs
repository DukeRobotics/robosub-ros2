import foxglove from "@foxglove/eslint-plugin";
import path from "node:path";
import { config } from "typescript-eslint";
import { includeIgnoreFile } from "@eslint/compat";
import { fileURLToPath } from "node:url";

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);
const ignorePath = path.resolve(__dirname, ".eslintignore");

export default config(includeIgnoreFile(ignorePath), {
  extends: [foxglove.configs.base, foxglove.configs.typescript],
  languageOptions: {
    parserOptions: {
      project: ["**/tsconfig.json"],
      tsconfigRootDir: __dirname,
    },
  },
});
