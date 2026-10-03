## Vendored dependency patches

This repository vendors third-party dependencies under `vendor/`.

When an upstream dependency needs a small fix (and maintaining a long-lived fork is
undesirable), we keep the fix as a patch file in-tree and apply it on top of the
vendored checkout.

### Layout

Patches are stored under:

- `deps/patches/vendor/<repo_name>/`

Where `<repo_name>` matches the directory name under `vendor/` (e.g.,
`vendor/tesseract_ros2`).

Patch files are applied in lexicographic order, so use a numeric prefix like
`0001-...patch`, `0002-...patch`, etc.

### Apply / unapply

Apply all patches:

```bash
python3 deps/scripts/apply_patches.py
```

Show status:

```bash
python3 deps/scripts/apply_patches.py --status
```

Unapply all patches:

```bash
python3 deps/scripts/apply_patches.py --reverse
```

