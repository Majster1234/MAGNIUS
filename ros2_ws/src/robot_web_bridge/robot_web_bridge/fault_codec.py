"""Bezstratne, zwarte kodowanie migawek awarii i plik rekordów `faults.rfs`.

Plik nie jest przeznaczony do czytania wprost; odczyt: `python3 -m robot_web_bridge.fault_snapshot --list|--dump`.

Rekord = `RFS1` + <u32 długość metadanych> + <u64 długość danych> + metadane JSON (bez kompresji: lista
i retencja bez dekompresji) + dane xz. Dane po dekompresji to wiersze JSON:

- wiersz 0: pełny nagłówek migawki (wyzwalacze, konfiguracja, parametry, sesja, …),
- `["O", source, offset]`: offset `mono_ns − utc_ns` źródła (pierwsze wystąpienie),
- `["K", id, source, event|null, has_event, has_recording_id, [klucze]]`: schemat wierszy,
- `[Δutc_ns, id, wartości…]`: wiersz; Δ względem poprzedniego wiersza (pierwszy: wartość bezwzględna).

Wartości całkowite są kodowane względnie: `mono_ns` → mono − utc − offset źródła, `sequence` → różnica do
poprzedniego numeru tego źródła, `request_ns`/`deadline_ns` → względem `mono_ns` wiersza, `kernel_utc_ns` →
względem `utc_ns`. `recording_id` równy identyfikatorowi sesji z nagłówka nie jest powtarzany.
"""
import json
import lzma
import os
from pathlib import Path
import struct

MAGIC = b'RFS1'
RECORD = struct.Struct('<4sIQ')
FORMAT = 1
MONO_RELATIVE = ('request_ns', 'deadline_ns')
UTC_RELATIVE = ('kernel_utc_ns',)
SECOND = 1_000_000_000


def _dumps(value):
    return json.dumps(value, ensure_ascii=False, separators=(',', ':'))


def _integer(value):
    return type(value) is int  # bool nie jest liczbą w tym kodowaniu


def encode_rows(rows, recording_id=''):
    """Słowniki z `source` i `utc_ns` (w kolejności czasu) → linie tekstu."""
    out, schemas, offsets, sequences, previous = [], {}, {}, {}, 0
    for original in rows:
        row = dict(original)
        source = row.pop('source')
        has_event = 'event' in row
        event = row.pop('event', None)
        utc = row.pop('utc_ns')
        has_recording = 'recording_id' in row and row['recording_id'] == recording_id
        if has_recording:
            row.pop('recording_id')
        mono = row.get('mono_ns')
        if _integer(mono):
            if source not in offsets:
                offsets[source] = mono - utc
                out.append(_dumps(['O', source, offsets[source]]))
            row['mono_ns'] = mono - utc - offsets[source]
            for key in MONO_RELATIVE:
                if _integer(row.get(key)):
                    row[key] -= mono
        for key in UTC_RELATIVE:
            if _integer(row.get(key)):
                row[key] -= utc
        if _integer(row.get('sequence')):
            sequence = row['sequence']
            row['sequence'] = sequence - sequences.get(source, 0)
            sequences[source] = sequence
        keys = tuple(row)
        signature = (source, event, has_event, has_recording, keys)
        if signature not in schemas:
            schemas[signature] = len(schemas)
            out.append(_dumps(['K', schemas[signature], source, event, has_event, has_recording, list(keys)]))
        out.append(_dumps([utc - previous, schemas[signature], *row.values()]))
        previous = utc
    return out


def decode_rows(lines, recording_id=''):
    rows, schemas, offsets, sequences, previous = [], {}, {}, {}, 0
    for line in lines:
        item = json.loads(line)
        if item and item[0] == 'O':
            offsets[item[1]] = item[2]
            continue
        if item and item[0] == 'K':
            schemas[item[1]] = item[2:]
            continue
        delta, schema, values = item[0], item[1], item[2:]
        source, event, has_event, has_recording, keys = schemas[schema]
        utc = previous + delta
        previous = utc
        row = dict(zip(keys, values))
        if _integer(row.get('mono_ns')) and source in offsets:
            row['mono_ns'] += utc + offsets[source]
            for key in MONO_RELATIVE:
                if _integer(row.get(key)):
                    row[key] += row['mono_ns']
        for key in UTC_RELATIVE:
            if _integer(row.get(key)):
                row[key] += utc
        if _integer(row.get('sequence')):
            row['sequence'] += sequences.get(source, 0)
            sequences[source] = row['sequence']
        result = {'source': source}
        if has_event:
            result['event'] = event
        if has_recording:
            result['recording_id'] = recording_id
        result.update(row)
        result['utc_ns'] = utc
        rows.append(result)
    return rows


def encode_snapshot(header, rows, preset=6):
    text = '\n'.join([_dumps(header), *encode_rows(rows, header.get('recording_id', ''))])
    return lzma.compress(text.encode('utf-8'), preset=preset)


def decode_snapshot(payload):
    lines = lzma.decompress(payload).decode('utf-8').split('\n')
    header = json.loads(lines[0])
    return header, decode_rows(lines[1:], header.get('recording_id', ''))


class RecordStore:
    """Plik rekordów migawek z retencją: zawsze co najmniej keep_s okien, najwyżej max_bytes."""

    def __init__(self, path, keep_s=900.0, max_bytes=256 * 1024 * 1024):
        self.path = Path(path)
        self.keep_s, self.max_bytes = keep_s, max_bytes

    def index(self):
        """[(offset, długość, metadane)] poprawnych rekordów i koniec ostatniego poprawnego rekordu."""
        entries, offset = [], 0
        try:
            with self.path.open('rb') as file:
                size = os.fstat(file.fileno()).st_size
                while offset + RECORD.size <= size:
                    file.seek(offset)
                    magic, meta_length, data_length = RECORD.unpack(file.read(RECORD.size))
                    length = RECORD.size + meta_length + data_length
                    if magic != MAGIC or offset + length > size:
                        break  # niepełny zapis na końcu (np. zanik zasilania)
                    try:
                        meta = json.loads(file.read(meta_length))
                    except ValueError:
                        break
                    entries.append((offset, length, meta))
                    offset += length
        except OSError:
            pass
        return entries, offset

    def load(self, offset):
        with self.path.open('rb') as file:
            file.seek(offset)
            _, meta_length, data_length = RECORD.unpack(file.read(RECORD.size))
            meta = json.loads(file.read(meta_length))
            return meta, file.read(data_length)

    def append(self, meta, payload):
        meta_bytes = _dumps(meta).encode('utf-8')
        record = RECORD.pack(MAGIC, len(meta_bytes), len(payload)) + meta_bytes + payload
        entries, valid_end = self.index()
        duration = sum(entry[2].get('duration_s', 0) for entry in entries) + meta.get('duration_s', 0)
        total = valid_end + len(record)
        first = 0
        while first < len(entries) and (duration - entries[first][2].get('duration_s', 0) >= self.keep_s
                                        or total > self.max_bytes):
            duration -= entries[first][2].get('duration_s', 0)
            total -= entries[first][1]
            first += 1
        size = self.path.stat().st_size if self.path.exists() else 0
        if first == 0 and size == valid_end:
            with self.path.open('ab') as file:
                file.write(record)
                file.flush()
                os.fsync(file.fileno())
            return
        # Usunięcie najstarszych lub uszkodzonego końca: kopia zachowanego zakresu i atomowa podmiana.
        temporary = self.path.with_name(self.path.name + '.tmp')
        start = entries[first][0] if first < len(entries) else valid_end
        with temporary.open('wb') as target:
            if size:
                with self.path.open('rb') as source:
                    source.seek(start)
                    remaining = valid_end - start
                    while remaining > 0:
                        chunk = source.read(min(remaining, 8 * 1024 * 1024))
                        if not chunk:
                            break
                        target.write(chunk)
                        remaining -= len(chunk)
            target.write(record)
            target.flush()
            os.fsync(target.fileno())
        temporary.replace(self.path)
