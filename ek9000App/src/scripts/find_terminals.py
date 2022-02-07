#!/usr/bin/env python3
"""
python3 find_terminals.py [--json] regex [regex [regex...]]
Where regex is a regular expression to match terminals.

Tool for getting PDO information out of the TwinCAT XML files.

Usage examples:

```bash
$ python3 find_terminals.py EL5042
$ python3 find_terminals.py EL504[0123]
$ python3 find_terminals.py EL5042 --json
```
"""

from __future__ import annotations

import dataclasses
import math
import pathlib
import json
import lxml
import lxml.etree
import re
import sys

from typing import Dict, Generator, List, Optional, Pattern, Tuple


HELP_DOCS = __doc__.strip()


def load_existing_terminals() -> Dict[str, dict]:
    with open("terminals.json") as fp:
        return {
            term["name"]: term
            for term in json.load(fp)["terminals"]
        }


def _single_text_or_none(obj: lxml.etree.Element, path: str) -> Optional[str]:
    items = obj.xpath(path)
    if len(items) > 1:
        raise RuntimeError(f"Expected only 0 or 1 elements in xpath: {items}")
    try:
        item, = items
    except ValueError:
        return None
    return item.text


def _from_hex(value: Optional[str]) -> Optional[int]:
    if value is None:
        return None
    value = value.strip()
    if value.startswith("#x"):
        return int(value.replace("#x", ""), 16)
    return int(value)


def _bits_to_words(bits: int) -> int:
    return int(math.ceil(bits / 16))


@dataclasses.dataclass
class PdoEntry:
    name: Optional[str]
    index: int
    subindex: Optional[int]
    bit_length: int
    data_type: str
    # element: lxml.etree.Element

    @classmethod
    def from_xml(cls, obj: lxml.etree.Element) -> PdoEntry:
        return cls(
            name=_single_text_or_none(obj, "Name"),
            data_type=_single_text_or_none(obj, "DataType"),
            bit_length=_from_hex(_single_text_or_none(obj, "BitLen")),
            index=_from_hex(_single_text_or_none(obj, "Index")),
            subindex=_from_hex(_single_text_or_none(obj, "SubIndex")),
            # element=obj,
        )


@dataclasses.dataclass
class PdoInfo:
    name: str
    index: int
    fixed: bool
    mandatory: bool
    entries: List[PdoEntry]
    # element: lxml.etree.Element

    @property
    def bit_length(self) -> int:
        """PDO size in bits."""
        if not self.entries:
            return 0
        return sum(entry.bit_length for entry in self.entries)

    @property
    def byte_length(self) -> int:
        """PDO size in bytes."""
        return int(math.ceil(self.bit_length / 8))

    @property
    def word_length(self) -> int:
        """PDO size in 16-bit words."""
        return int(math.ceil(self.bit_length / 16))

    @property
    def summary(self) -> str:
        desc = " ".join(
            (
                ("mandatory" if self.mandatory else ""),
                ("fixed" if self.fixed else ""),
            )
        ).strip()
        return f"{self.name} ({self.word_length} words @ {self.index} {desc})"

    @classmethod
    def from_xml(cls, obj: lxml.etree.Element) -> PdoInfo:
        name = obj.xpath("Name")[0].text
        index = _from_hex(obj.xpath("Index")[0].text)
        return cls(
            name=name,
            index=index,
            # element=obj,
            fixed=obj.attrib.get("Fixed", None) == "1",
            mandatory=obj.attrib.get("Mandatory", None) == "1",
            entries=[PdoEntry.from_xml(entry) for entry in obj.xpath("Entry")],
        )


@dataclasses.dataclass
class SlaveMappings:
    name: str
    output_indices: List[int]  # SM2
    input_indices: List[int]  # SM3

    @classmethod
    def from_xml(cls, obj: lxml.etree.Element) -> SlaveMappings:
        def get_pdos(number):
            return [_from_hex(pdo.text) for pdo in obj.xpath(f"Sm[@No={number}]/Pdo")]
        return cls(
            name=_single_text_or_none(obj, "Name"),
            output_indices=get_pdos(2),
            input_indices=get_pdos(3),
        )


@dataclasses.dataclass
class TerminalInfo:
    name: str
    revision: str
    # element: lxml.etree.Element
    rxpdo: List[PdoInfo]
    txpdo: List[PdoInfo]
    mappings: List[SlaveMappings]

    def pdo_by_index(self, index: int, rx: bool) -> PdoInfo:
        pdos = self.rxpdo if rx else self.txpdo
        for pdo in pdos:
            if pdo.index == index:
                return pdo
        raise ValueError("Unknown index")

    @classmethod
    def from_xml(cls, obj: lxml.etree.Element) -> TerminalInfo:
        txpdos = [PdoInfo.from_xml(pdo) for pdo in obj.xpath("TxPdo")]
        rxpdos = [PdoInfo.from_xml(pdo) for pdo in obj.xpath("RxPdo")]
        mappings = [
            SlaveMappings(
                name="All PDOs",
                output_indices=[pdo.index for pdo in rxpdos],
                input_indices=[pdo.index for pdo in txpdos],
            )
        ]
        mappings += [
            SlaveMappings.from_xml(sm)
            for sm in obj.xpath("Info/VendorSpecific/TwinCAT/AlternativeSmMapping")
        ]
        return TerminalInfo(
            name=obj.xpath("Type")[0].text,
            revision=obj.xpath("Type")[0].attrib.get("RevisionNo", ""),
            txpdo=txpdos,
            rxpdo=rxpdos,
            mappings=mappings,
            # element=obj,
        )

    def size_for_mapping(self, mapping: SlaveMappings) -> Tuple[List[int], List[int]]:
        """(input, output) size in words for the mapping."""
        input_words = [
            self.pdo_by_index(input_index, rx=False).word_length
            for input_index in mapping.input_indices
        ]
        output_words = [
            self.pdo_by_index(input_index, rx=True).word_length
            for input_index in mapping.output_indices
        ]
        return input_words, output_words


def find_matching(regex: Pattern) -> Generator[TerminalInfo, None, None]:
    for fn in pathlib.Path("BhcConfigFiles").glob("*.xml"):
        with open(fn, 'rb') as f:
            tree = lxml.etree.parse(f)

        devices = tree.xpath("Descriptions/Devices/Device")

        for dev in devices:
            dev_name = dev.xpath("Type")[0].text
            if regex.match(dev_name):
                yield TerminalInfo.from_xml(dev)


def main(names_to_match: List[str], *, dump_json: bool = False):
    regex = re.compile("|".join(names_to_match), flags=re.IGNORECASE)
    existing_terminals = load_existing_terminals()
    for match_idx, info in enumerate(find_matching(regex)):
        if dump_json:
            print(json.dumps(dataclasses.asdict(info), skipkeys=["element"]))
            continue

        if match_idx > 0:
            print()

        print(info.name, "rev", info.revision)
        if info.rxpdo:
            print("  - RxPdo")
            for idx, pdo in enumerate(info.rxpdo):
                print(f"   {idx}. {pdo.summary}")
                for entry in pdo.entries:
                    print(f"      - {entry.name}: {entry.bit_length} bits ({entry.data_type})")

        if info.txpdo:
            print("  - TxPdo")
            for idx, pdo in enumerate(info.txpdo):
                print(f"   {idx}. {pdo.summary}")
                for entry in pdo.entries:
                    print(f"      - {entry.name}: {entry.bit_length} bits ({entry.data_type})")

        existing_info = existing_terminals.get(info.name)
        for mapping in info.mappings:
            input_size, output_size = info.size_for_mapping(mapping)
            print(f"   * Mapping {mapping.name!r}")

            total_input_size = sum(input_size or [0])
            total_output_size = sum(output_size or [0])
            print(
                f"     pdo_in_size={total_input_size} entries={input_size} "
                f"pdo_out_size={total_output_size} entries={output_size}",
                end=" / "
            )

            if existing_info is None:
                print("not in terminals.json")
                continue

            to_compare = [
                ("inputs", len(input_size)),
                ("outputs", len(output_size)),
                ("pdo_in_size", total_input_size),
                ("pdo_out_size", total_output_size),
            ]

            differences = " ".join(
                f"{key}={existing_info[key]}"
                for key, calculated in to_compare
                if existing_info[key] != calculated
            )
            if differences:
                print(f"terminals.json differs: {differences}")
            else:
                print("terminals.json **matches**")


if __name__ == "__main__":
    names_to_match = sys.argv[1:]
    dump_json = "--json" in names_to_match
    if dump_json:
        names_to_match.remove("--json")
    if not names_to_match:
        print(HELP_DOCS)
        sys.exit(1)

    main(names_to_match, dump_json=dump_json)
