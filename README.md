# MRS Documentation Repo

Source code for documentation written by MRS members for MRS members. Documentation is hosted using [MKDocs](https://www.mkdocs.org/)  with the [Material theme](https://squidfunk.github.io/mkdocs-material/)

# Contributing
See contributing guidelines [here](./CONTRIBUTING.md)

## Setup
Download dependencies with  
```
python3 -m pip install --upgrade mkdocs mkdocs-material
```
Clone the **DEV** branch (**DO NOT CLONE MASTER**)

## Adding/changing documentation
Pages are written in markdown (`.md`) and are found in the `docs` directory.

Markdown formatting guide can be found [here](https://docs.github.com/en/get-started/writing-on-github/getting-started-with-writing-and-formatting-on-github/basic-writing-and-formatting-syntax).

MKDocs Material theme also supports extra extra extensions to make documentation easier to read or just nicer to look at. Information on this can be found [on the MKDocs Material page](https://squidfunk.github.io/mkdocs-material/reference/).

If you are adding pages or changing the organisation of the documentation site, update `mkdocs.yml` to reflect the new site organisation. Read the [documentation here](https://squidfunk.github.io/mkdocs-material/creating-your-site/) to see how to update `mkdocs.yml`

Once you are satisfied with your changes, commit them to the **DEV** branch (**NOT THE MASTER BRANCH**) and EXCO Logs will review the commit before merging it