# User Manual

This folder hosts the source used for generating the user manual.

The user manual is generated using `mdbook`, which can be installed via cargo:
```
cargo install mdbook
cargo install mdbook-toc
cargo install mdbook-linkcheck
```

To build the user manual locally and then serve the book:
```
mdbook serve book
```

Once the `mdbook serve` command is run, the manual can be found on a web browser at
`localhost:3000`.
