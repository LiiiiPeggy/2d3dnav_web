"""Small PCD reader for FAST-LIO's ASCII or uncompressed-binary maps."""

import numpy as np


def load_xyz(path):
    header = {}
    with open(path, 'rb') as stream:
        while True:
            line = stream.readline()
            if not line:
                raise ValueError('PCD header has no DATA entry')
            decoded = line.decode('ascii', errors='strict').strip()
            if not decoded or decoded.startswith('#'):
                continue
            parts = decoded.split()
            header[parts[0].upper()] = parts[1:]
            if parts[0].upper() == 'DATA':
                break

        fields = header.get('FIELDS') or header.get('FIELD')
        sizes = [int(value) for value in header.get('SIZE', [])]
        types = header.get('TYPE', [])
        counts = [int(value) for value in header.get('COUNT', ['1'] * len(fields))]
        if not fields or len(fields) != len(sizes) or len(fields) != len(types):
            raise ValueError('invalid PCD FIELDS/SIZE/TYPE header')
        if len(counts) != len(fields):
            raise ValueError('invalid PCD COUNT header')
        if not all(axis in fields for axis in ('x', 'y', 'z')):
            raise ValueError('PCD must contain x, y and z fields')

        point_count = int((header.get('POINTS') or ['0'])[0])
        data_kind = header['DATA'][0].lower()
        if data_kind == 'ascii':
            values = np.loadtxt(stream, dtype=np.float32, ndmin=2)
            offsets = np.cumsum([0] + counts[:-1]).tolist()
            return np.column_stack([
                values[:, offsets[fields.index(axis)]] for axis in ('x', 'y', 'z')
            ]).astype(np.float32, copy=False)
        if data_kind != 'binary':
            raise ValueError(
                f"unsupported PCD DATA {data_kind!r}; use ASCII or binary, not binary_compressed")

        type_codes = {
            ('F', 4): '<f4', ('F', 8): '<f8',
            ('I', 1): '<i1', ('I', 2): '<i2', ('I', 4): '<i4', ('I', 8): '<i8',
            ('U', 1): '<u1', ('U', 2): '<u2', ('U', 4): '<u4', ('U', 8): '<u8',
        }
        dtype_fields = []
        for name, size, value_type, count in zip(fields, sizes, types, counts):
            code = type_codes.get((value_type.upper(), size))
            if code is None:
                raise ValueError(f'unsupported PCD field type {value_type}{size}')
            dtype_fields.append((name, code) if count == 1 else (name, code, (count,)))
        values = np.fromfile(stream, dtype=np.dtype(dtype_fields), count=point_count or -1)
        return np.column_stack([values[axis] for axis in ('x', 'y', 'z')]).astype(
            np.float32, copy=False)
