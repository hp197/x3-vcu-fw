"""PlatformIO post-script: count the slot-0 sections x3-vcu-fw.ld adds in the flash size check.

The platform's size regexp lists .isr_vector, .text, .data, .rodata and a few more, but not
.text_low (the code between the vector table and 0x08001400) or .app_header (the slot-0
banner, key and build id at 0x08001400). Without them the reported flash use, and the
board_upload.maximum_size check, came out about 700 bytes low.
"""

Import("env")  # noqa: F821  (SCons global)

env.Replace(  # noqa: F821
    SIZEPROGREGEXP=r"^(?:\.isr_vector|\.text_low|\.app_header|\.text|\.data|\.rodata|\.text.align"
    r"|\.init_array|\.fini_array|\.ARM.exidx)\s+(\d+).*"
)
