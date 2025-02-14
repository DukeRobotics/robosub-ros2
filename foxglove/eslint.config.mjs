import foxglove from "@foxglove/eslint-plugin";
import path from "node:path";
import { config } from "typescript-eslint";
import { includeIgnoreFile } from "@eslint/compat";
import { fileURLToPath } from "node:url";

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);
const ignorePath = path.resolve(__dirname, "eslintignore");

export default config(includeIgnoreFile(ignorePath), {
  extends: [foxglove.configs.base, foxglove.configs.react, foxglove.configs.typescript],
  languageOptions: {
    parserOptions: {
      project: ["**/tsconfig.json"],
      tsconfigRootDir: __dirname,
    },
  },
  rules: {
    "no-warning-comments": ["warn"],
    camelcase: [
      "warn",
      {
        properties: "always",
      },
    ],
    "react-hooks/exhaustive-deps": ["error"],
    "prettier/prettier": [
      "error",
      {
        endOfLine: "lf",
      },
    ],
    "spaced-comment": [
      "error",
      "always",
      {
        block: {
          balanced: true,
        },
      },
    ],
    "capitalized-comments": ["warn", "always", { ignoreConsecutiveComments: true }],
  },
});
