import re
from pathlib import Path

Import("env")


COMMON_SAMPLE_RATES = [
    44100,
    48000,
    88200,
    96000,
    176400,
    192000,
]

# 2 channels * 24-bit samples = 6 bytes per frame
MAX_PACKET_BYTES = (max(COMMON_SAMPLE_RATES) * 6) // 1000


def _build_format_descriptor_block() -> str:
    count = len(COMMON_SAMPLE_RATES)
    length = 8 + (count * 3)
    lines = [
        f"\t{length},\t\t\t\t\t// bLength",
        "\t0x24,\t\t\t\t\t// bDescriptorType = CS_INTERFACE",
        "\t2,\t\t\t\t\t// bDescriptorSubtype = FORMAT_TYPE",
        "\t1,\t\t\t\t\t// bFormatType = FORMAT_TYPE_I",
        "\t2,\t\t\t\t\t// bNrChannels = 2",
        "\t3,\t\t\t\t\t// bSubFrameSize = 3 bytes",
        "\t24,\t\t\t\t\t// bBitResolution = 24 bits",
        f"\t{count},\t\t\t\t\t// bSamFreqType = {count} frequencies",
    ]
    for i, rate in enumerate(COMMON_SAMPLE_RATES):
        lines.append(f"\tLSB({rate}), MSB({rate}), 0,\t\t// tSamFreq[{i}]")
    return "\n".join(lines) + "\n"


def _replace_exact(text: str, old: str, new: str) -> tuple[str, bool]:
    if old in text:
        return text.replace(old, new), True
    return text, False


def patch_usb_desc_c(path: Path) -> list[str]:
    text = path.read_text(encoding="utf-8")
    changed = []

    # Keep descriptor size consistent with the current number of sample-rate entries.
    format_len = 8 + (len(COMMON_SAMPLE_RATES) * 3)
    size_line = (
        "#define AUDIO_INTERFACE_DESC_SIZE\t8 + 9+10+12+9+12+10+9 + "
        f"9+9+7+{format_len}+9+7 + 9+9+7+{format_len}+9+7+9"
    )
    text2, size_n = re.subn(
        r"(?m)^#define\s+AUDIO_INTERFACE_DESC_SIZE\s+8\s*\+\s*9\+10\+12\+9\+12\+10\+9\s*\+\s*9\+9\+7\+\d+\+9\+7\s*\+\s*9\+9\+7\+\d+\+9\+7\+9\s*$",
        size_line,
        text,
        count=1,
    )
    if size_n > 0:
        text = text2
        changed.append("AUDIO_INTERFACE_DESC_SIZE")

    # Canonical Type I format descriptor payload for all common sample rates.
    canonical = _build_format_descriptor_block()

    pattern = re.compile(
        r"(?ms)(\s*// Type I Format Descriptor\s*\n\s*// USB DCD for Audio Data Formats 1\.0, Section 2\.2\.5, Table 2-1, page 10\s*\n)"
        r"(.*?)"
        r"(\s*// Standard AS Isochronous Audio Data Endpoint Descriptor)",
    )

    def repl(match: re.Match[str]) -> str:
        return f"{match.group(1)}{canonical}{match.group(3)}"

    text2, n = pattern.subn(repl, text)
    if n >= 2 and text2 != text:
        text = text2
        changed.append(f"TypeI descriptors ({n})")

    if changed:
        path.write_text(text, encoding="utf-8")
    return changed


def patch_usb_desc_h(path: Path) -> list[str]:
    text = path.read_text(encoding="utf-8")
    changed = []

    # Restrict size edits to USB_MIDI_AUDIO_SERIAL block.
    block_pattern = re.compile(
        r"(?ms)(#elif defined\(USB_MIDI_AUDIO_SERIAL\).*?)(\n#elif defined\(|\n#endif)",
    )
    m = block_pattern.search(text)
    if not m:
        return changed

    block = m.group(1)
    block_new = re.sub(
        r"(#define\s+AUDIO_TX_SIZE\s+)\d+",
        rf"\g<1>{MAX_PACKET_BYTES}",
        block,
    )
    block_new = re.sub(
        r"(#define\s+AUDIO_RX_SIZE\s+)\d+",
        rf"\g<1>{MAX_PACKET_BYTES}",
        block_new,
    )

    if block_new != block:
        text = text[: m.start(1)] + block_new + text[m.end(1) :]
        changed.append(f"USB_MIDI_AUDIO_SERIAL AUDIO_{{TX,RX}}_SIZE={MAX_PACKET_BYTES}")
        path.write_text(text, encoding="utf-8")

    return changed


def main() -> None:
    framework_dir = env.PioPlatform().get_package_dir("framework-arduinoteensy")
    if not framework_dir:
        print("[usb-audio patch] framework-arduinoteensy not found, skipping")
        return

    core = Path(framework_dir) / "cores" / "teensy4"
    c_path = core / "usb_desc.c"
    h_path = core / "usb_desc.h"

    if not c_path.exists() or not h_path.exists():
        print("[usb-audio patch] teensy4 usb_desc files not found, skipping")
        return

    changes = []
    changes.extend(patch_usb_desc_c(c_path))
    changes.extend(patch_usb_desc_h(h_path))

    if changes:
        print("[usb-audio patch] applied: " + ", ".join(changes))
    else:
        print("[usb-audio patch] already up to date")


main()
