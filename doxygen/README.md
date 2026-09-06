# Documentation site (Doxygen)

The whole Solaris Software site **is** Doxygen: the narrative guides and the C
API reference are one generated site, themed to match the old MkDocs look
(dark, JetBrains Mono, doxygen-awesome-css).

## Layout

| Path | What |
|------|------|
| `../Doxyfile`        | configuration |
| `pages/`             | the narrative guides, as Markdown (`*.md`); `pages/index.md` is the front page |
| `pages/assets/`      | images used by those pages |
| `header.html` / `footer.html` | Doxygen 1.9.8 templates, wired for the theme and forced dark |
| `solaris.css`        | palette / font overrides + search-box placement |
| `theme/`             | vendored [doxygen-awesome-css](https://github.com/jothepro/doxygen-awesome-css) v2.3.4 (MIT, see `theme/LICENSE`) |
| `assets/`            | logo, favicon, font copied into the output |

The API side is generated from the source comments in `solaris-v2/main` and
`solaris-v2/spp` (see `INPUT` in the Doxyfile).

## Adding / editing a guide

1. Add or edit `pages/<name>.md`. Start it with `# Title   {#name}` so the
   page has a stable id.
2. Link it into the nav: add `@subpage <name>` to the relevant parent page
   (`index.md`, `start-here.md`, `repositories.md`, `spp-arch.md`,
   `spp-detail.md`).
3. Put images under `pages/assets/` and reference them by bare filename:
   `![alt](foo.svg)`.

## Building locally

From the repository root, with `doxygen` (>= 1.9.8) and `graphviz`:

```bash
doxygen Doxyfile          # output in doc/html/ ; open doc/html/index.html
```

`doc/` is git-ignored.

## CI

`.github/workflows/deploy-website.yml` rebuilds the site in the `solaris-ci`
container on **every push to `main`** (and on manual dispatch) and rsyncs
`doc/html/` to the web server.

## Notes

- Dark-only: `header.html` ships `<html class="dark-mode">` and `solaris.css`
  pins the palette; `HTML_COLORSTYLE` stays `LIGHT` (doxygen-awesome needs it).
- `DISABLE_INDEX = YES` so the search box sits in the title bar (with
  `DISABLE_INDEX = NO` the sidebar layout was hiding it).
- `*/README.md` and `*/LICENSE.md` are excluded so stray source-tree markdown
  doesn't turn into pages.
- Source doc-comment warnings (mismatched `@param`, etc.) are pre-existing and
  don't fail the build (`WARN_AS_ERROR = NO`).
