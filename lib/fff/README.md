# fff

The Fake Function Framework: a header-only C library that generates fake functions with
call counts, argument history, return-value sequences and custom implementations. The
on-target unit tests (`pio test -e emu-test`, see [test/README.md](../../test/README.md))
use it for the bookkeeping of the recording fakes they put at the Artery BSP boundary with
GNU ld `-Wl,--wrap=<function>`. Nothing in the product build includes it.

## Upstream

- Repository: <https://github.com/meekrosoft/fff>
- Commit: `5111c61e1ef7848e3afd3550044a8cf4405f4199` (2023-05-22, "Add GTest with
  FetchContent (#120)"), the head of `master` on 2026-11-06.
- Licence: MIT, copied here as [LICENSE](LICENSE) (the same text heads `fff.h`).

| File here | Upstream file | sha256 |
|---|---|---|
| `fff.h` | `fff.h` | `f0dc1aa189dbbb771078a1657be6c6aa1943cb4fbc6b3b5c0970f8608f97a887` |
| `LICENSE` | `LICENSE` | `55c519a17786c1eccd2ba85de9240330b203e255a1082cf79753b3b5f6d6c0bd` |

## Local changes

None: both files are unmodified copies. Configuration (`FFF_ARG_HISTORY_LEN`,
`FFF_CALL_HISTORY_LEN`) is set from `platformio.ini` (`[env:emu-test]`), not by editing
the header.
