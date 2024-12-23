import { FlatCompat } from "@eslint/eslintrc";
import js from "@eslint/js";
import jest from "eslint-plugin-jest";
import globals from "globals";
import path from "node:path";
import { fileURLToPath } from "node:url";

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);
const compat = new FlatCompat({
  baseDirectory: __dirname,
  recommendedConfig: js.configs.recommended,
  allConfig: js.configs.all,
});

export default [
  {
    ignores: ["**/dist"],
  },
  ...compat.extends("plugin:@foxglove/base", "plugin:@foxglove/react"),
  {
    plugins: {
      jest,
    },

    languageOptions: {
      globals: {
        ...globals.browser,
        ...Object.fromEntries(Object.entries(globals.node).map(([key]) => [key, "off"])),
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
          endOfLine: "auto",
        },
      ],

      "spaced-comment": [
        "error",
        "always",
        {
          line: {
            markers: ["/"],
            exceptions: ["-", "+"],
          },

          block: {
            markers: ["!"],
            exceptions: ["*"],
            balanced: true,
          },
        },
      ],

      "capitalized-comments": ["warn", "always"],
    },
  },
  ...compat.extends("plugin:@foxglove/typescript").map((config) => ({
    ...config,
    files: ["**/*.ts", "**/*.tsx"],
  })),
  {
    files: ["**/*.ts", "**/*.tsx"],

    languageOptions: {
      ecmaVersion: 5,
      sourceType: "script",

      parserOptions: {
        project: "**/tsconfig.json",
      },
    },
  },
];
