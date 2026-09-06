# Doxygen API reference

The C API reference for `solaris-v2` (every file, function, struct and macro,
pulled from the source comments). It is themed to match the Solaris
documentation site and is published as the **API (Doxygen)** section of it.

## Layout

| Path | What |
|------|------|
| `../Doxyfile`        | configuration (`INPUT = mainpage.dox solaris-v2/main solaris-v2/spp`) |
| `../mainpage.dox`    | the landing page |
| `header.html` / `footer.html` | Doxygen 1.9.8 templates, wired for the theme and forced dark |
| `solaris.css`        | palette / font overrides (Gruvbox dark, JetBrains Mono) |
| `theme/`             | vendored [doxygen-awesome-css](https://github.com/jothepro/doxygen-awesome-css) v2.3.4 (MIT, see `theme/LICENSE`) |
| `assets/`            | logo, favicon, font copied into the output |

## Generating it locally

From the repository root, with `doxygen` (>= 1.9.8) and `graphviz` installed:

```bash
doxygen Doxyfile          # output in doc/html/ ; open doc/html/index.html
```

`doc/` is git-ignored.

## CI

`.github/workflows/deploy-website.yml` regenerates this in the `solaris-ci`
container, copies `doc/html/` into `website/site/doxygen/`, and deploys it with
the rest of the site. It runs on pushes to `website/**`, `doxygen/**`,
`Doxyfile` or `mainpage.dox`, and on manual dispatch.

## Notes

- The site is dark-only, so `header.html` ships `<html class="dark-mode">` and
  `solaris.css` pins the dark palette; `HTML_COLORSTYLE` stays `LIGHT` because
  doxygen-awesome-css requires it.
- Source doc-comment warnings (mismatched `@param`, stale `\file` names, etc.)
  are pre-existing and don't fail the build (`WARN_AS_ERROR = NO`).
