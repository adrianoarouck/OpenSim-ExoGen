"""Minimal reader for VTK XML PolyData (.vtp) meshes, no VTK dependency.

Supports inline ``ascii`` and ``binary`` (base64, optionally zlib-compressed)
DataArrays, which covers the OpenSim Geometry folder. Polygons are
fan-triangulated. Returns flat lists ready for a Three.js BufferGeometry.
"""
import base64
import struct
import xml.etree.ElementTree as ET
import zlib

import numpy as np

_DTYPES = {"Float32": "<f4", "Float64": "<f8", "Int32": "<i4", "Int64": "<i8", "UInt32": "<u4",
           "UInt64": "<u8", "UInt8": "<u1", "Int8": "<i1"}


def _read_array(elem, compressed, header_type):
    fmt = elem.get("format", "ascii")
    dtype = np.dtype(_DTYPES[elem.get("type")])
    text = (elem.text or "").strip()
    if fmt == "ascii":
        return np.array(text.split(), dtype=dtype) if text else np.zeros(0, dtype)
    if fmt != "binary":
        raise ValueError(f"unsupported DataArray format {fmt!r} (appended data)")
    hsize = 8 if header_type == "UInt64" else 4
    hfmt = "<Q" if hsize == 8 else "<I"
    raw = base64.b64decode(text)
    if not compressed:
        n = struct.unpack(hfmt, raw[:hsize])[0]
        return np.frombuffer(raw[hsize:hsize + n], dtype=dtype)
    # compressed: header = [nblocks, blocksize, lastsize, csize_1..n], base64-encoded on its own
    nblocks = struct.unpack(hfmt, base64.b64decode(text[:8])[:hsize])[0]
    hlen = hsize * (3 + nblocks)
    hb64 = 4 * ((hlen + 2) // 3)
    header = struct.unpack("<" + hfmt[1] * (3 + nblocks), base64.b64decode(text[:hb64])[:hlen])
    data = base64.b64decode(text[hb64:])
    out, pos = [], 0
    for csize in header[3:]:
        out.append(zlib.decompress(data[pos:pos + csize]))
        pos += csize
    return np.frombuffer(b"".join(out), dtype=dtype)


def read_vtp(path):
    root = ET.parse(path).getroot()
    compressed = root.get("compressor") is not None
    header_type = root.get("header_type", "UInt32")
    piece = root.find("PolyData/Piece")
    points = _read_array(piece.find("Points/DataArray"), compressed, header_type).reshape(-1, 3)
    normals = None
    for arr in piece.findall("PointData/DataArray"):
        if arr.get("Name", "").lower() == "normals":
            normals = _read_array(arr, compressed, header_type).reshape(-1, 3)
    conn = offs = None
    for arr in piece.findall("Polys/DataArray"):
        if arr.get("Name") == "connectivity":
            conn = _read_array(arr, compressed, header_type).astype(np.int64)
        elif arr.get("Name") == "offsets":
            offs = _read_array(arr, compressed, header_type).astype(np.int64)
    tris = []
    if conn is not None and offs is not None:
        start = 0
        for end in offs:
            poly = conn[start:end]
            for k in range(1, len(poly) - 1):
                tris.append((poly[0], poly[k], poly[k + 1]))
            start = end
    return {
        "positions": points.astype(np.float32).ravel().round(5).tolist(),
        "normals": None if normals is None else normals.astype(np.float32).ravel().round(4).tolist(),
        "indices": np.asarray(tris, dtype=np.int64).ravel().tolist(),
    }
