# ----------------------------------------------------------------------------
# -                        Open3D: www.open3d.org                            -
# ----------------------------------------------------------------------------
# The MIT License (MIT)
#
# Copyright (c) 2018-2021 www.open3d.org
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
# IN THE SOFTWARE.
# ----------------------------------------------------------------------------
"""Summary writer for the TensorBoard Open3D plugin"""
import threading
import os
import socket
import time
import queue
import warnings

import numpy as np

from tensorboard.compat.proto.summary_pb2 import Summary
from tensorboard.compat.proto.tensor_shape_pb2 import TensorShapeProto
from tensorboard.compat.proto.tensor_pb2 import TensorProto
from tensorboard.backend.event_processing.plugin_asset_util import PluginDirectory
from tensorboard.compat.tensorflow_stub.pywrap_tensorflow import masked_crc32c
from tensorboard.util import lazy_tensor_creator
try:
    import tensorflow as tf
    from tensorflow.experimental import dlpack as tf_dlpack
    from tensorflow.io.gfile import makedirs as _makedirs
    from tensorflow.io.gfile import GFile as _fileopen
except ImportError:
    tf = None
    from os import makedirs
    from functools import partial
    # Suppress errors for existing folders.
    _makedirs = partial(makedirs, exist_ok=True)
    _fileopen = open
try:
    import torch
    from torch.utils import dlpack as torch_dlpack
    from torch.utils.tensorboard import SummaryWriter
except ImportError:
    torch = None

import open3d as o3d
from open3d.visualization.tensorboard_plugin import plugin_data_pb2
from open3d.visualization.tensorboard_plugin import metadata
from open3d.visualization.tensorboard_plugin.util import _log


class _AsyncDataWriter:
    """ Write binary data to file asynchronously. Data buffers and files are
    queued with ``enqueue()`` and actual writing is done in a separate
    thread. GFile (``tf.io.gfile``) is used for writing to local and remote
    (Google cloud storage with gs:// URI and HDFS with hdfs:// URIs) locations.
    If tensorflow is not available, we fallback to Python I/O. The filename
    format is ``{tagfilepath}.{current time (s)}.{hostname}.{ProcessID}{filename_extension}``
    following the TensorFlow event file name format.

    This class is thread safe. A single global object is created when this
    module is imported by each process.
    """

    def __init__(self,
                 max_queue=10,
                 flush_secs=120,
                 filename_extension='.msgpack'):
        """
        Args:
            max_queue (int): enqueue will block if more than ``max_queue``
                writes are pending.
            flush_secs (Number): Data is flushed to disk / network periodically
                with this interval. Note that the data may still be in an OS
                buffer and not on disk.
            filename_extension (str): Extension for binary file.
        """
        self._max_queue = max_queue
        self._flush_secs = flush_secs
        self._next_flush_time = time.time() + self._flush_secs
        self._filename_extension = filename_extension
        self._write_queue = queue.Queue(maxsize=self._max_queue)
        self._file_handles = dict()
        self._file_next_write_pos = dict()
        # protects _file_handles and _file_next_write_pos
        self._file_lock = threading.Lock()
        self._writer_thread = threading.Thread()

    def _writer(self):
        """Writer thread main function."""
        while True:
            try:
                tagfilepath, data = self._write_queue.get(timeout=0.25)
            except queue.Empty:  # exit if nothing to do.
                break
            _log.debug(
                f"Writing {len(data)}b data at "
                f"{tagfilepath}+{self._file_handles[tagfilepath].tell()}")
            with self._file_lock:
                handle = self._file_handles[tagfilepath]
            handle.write(data)  # release lock for expensive I/O op
            self._write_queue.task_done()
            if time.time() > self._next_flush_time:
                file_handle_iter = iter(self._file_handles.values())
                try:
                    while True:
                        with self._file_lock:
                            handle = next(file_handle_iter)
                        handle.flush()  # release lock for expensive I/O op
                except (StopIteration, RuntimeError):
                    # RuntimeError: possible race condition in dict iterator,
                    # but PEP3106 guarantees no coruption. Try again later.
                    pass
                self._next_flush_time += self._flush_secs

    def enqueue(self, tagfilepath, data):
        """Add a write job to the write queue.

        Args:
            tagfilepath (str): Full file pathname for data. A suffix will be
                added to get the complete filename. A None value indicates
                writing is over and the writer thread should join.
            data (bytes): Data buffer to write.

        Returns:
            Tuple of filename and location (in bytes) where the data will be
            written.
        """
        with self._file_lock:
            if tagfilepath not in self._file_handles:
                # summary.writer.EventFileWriter name format
                fullfilepath = "{}.{}.{}.{}{}".format(tagfilepath,
                                                      int(time.time()),
                                                      socket.gethostname(),
                                                      os.getpid(),
                                                      self._filename_extension)
                _makedirs(os.path.dirname(fullfilepath))
                self._file_handles[tagfilepath] = _fileopen(fullfilepath, 'wb')
                _log.debug(f"msgpack file {fullfilepath} opened for writing.")
                this_write_loc = 0
                self._file_next_write_pos[tagfilepath] = len(data)
            else:
                this_write_loc = self._file_next_write_pos[tagfilepath]
                self._file_next_write_pos[tagfilepath] += len(data)
                fullfilepath = self._file_handles[tagfilepath].name
        # Blocks till queue has available slot.
        self._write_queue.put((tagfilepath, data), block=True)
        if not self._writer_thread.is_alive():
            self._writer_thread = threading.Thread(target=self._writer,
                                                   name="Open3DDataWriter")
            self._writer_thread.start()

        return os.path.basename(fullfilepath), this_write_loc


# Single global writer per process
_async_data_writer = _AsyncDataWriter()


def _to_o3d(tensor, min_ndim=None, max_len=None):
    """Convert Tensorflow, PyTorch and Numpy tensors to Open3D tensor without
    copying. Python lists and tuples are also accepted, but will be copied.

    Args:
        tensor: Input tensor to be converted.
        min_ndim (int): Tensor shape will be padded with ones on the left to
            reach this minimum number of dimensions.
        max_len (int): The max size along the first dimension. Other data will
            be discarded.
    """
    if isinstance(tensor, o3d.core.Tensor):
        pass
    elif tf is not None and isinstance(tensor, tf.Tensor):
        tensor = o3d.core.Tensor.from_dlpack(tf_dlpack.to_dlpack(tensor))
    elif torch is not None and isinstance(tensor, torch.Tensor):
        tensor = o3d.core.Tensor.from_dlpack(torch_dlpack.to_dlpack(tensor))
    else:
        tensor = o3d.core.Tensor.from_numpy(np.asarray(tensor))
    exp_shape = tensor.shape
    for _ in range(tensor.ndim, min_ndim):
        exp_shape.insert(0, 1)
    return tensor.reshape(exp_shape)[:max_len]


def _color_to_uint8(color_data):
    """
    Args:
        color_data: o3d.core.Tensor (B,N,3) with any dtype. Float dtypes are
        expected to have values in [0,1] and 8 bit Int dtypes in [0,255] and 16
        bit Int types in [0,2^16-1]

    Returns:
        color_data with the same shape, but as uint8 dtype.
    """
    if color_data.dtype == o3d.core.uint8:
        return color_data
    if color_data.dtype == o3d.core.uint16:
        return (color_data // 256).to(dtype=o3d.core.uint8)
    return (255 * color_data.clip(0, 1)).to(dtype=o3d.core.uint8)


def _to_integer(tensor):
    """Test converting a tensor (TF, PyTorch, Open3D, Numpy array or a scalar)
    to scalar integer (np.int64) and return it. Return None on failure.
    """
    try:
        if hasattr(tensor, 'ndim') and tensor.ndim > 0:
            return None
        if hasattr(tensor, 'dtype') and 'int' not in repr(tensor.dtype).lower():
            return None
        if hasattr(tensor, 'numpy'):
            tensor_int = tensor.numpy().astype(np.int64)
        tensor_int = np.int64(tensor)
        return tensor_int if tensor_int.size == 1 else None
    except (TypeError, ValueError, RuntimeError):
        return None


def _preprocess(prop,
                tensor,
                step,
                max_outputs,
                geometry_metadata,
                material=None):
    """Data conversion (format and dtype), validation and other preprocessing.
    Step reference support.
    TODO(ssheorey): Convert to half precision, compression, etc.
    """

    if prop == "material_name":
        if isinstance(tensor, (bytes, str)):
            material["name"] = (tensor,)
        elif isinstance(tensor[0], (bytes, str)):
            material["name"] = tensor
        else:
            raise ValueError(
                f"Value of the key `material_name` should be a str, bytes or"
                f" Sequence of str or bytes. Got {tensor} instead.")
        return material["name"]
    if prop.startswith("material_scalar_"):
        save_tensor = _to_o3d(tensor, min_ndim=1, max_len=max_outputs)
        if save_tensor.ndim != 1:
            raise ValueError(f"Material scalar property {prop} has shape "
                             f"{tensor.shape} instead of (B,)")
        material["scalar_properties"][prop[16:]] = save_tensor
        return save_tensor

    # Check if property is reference to prior step (not allowed for
    # material_name and material_scalar_*)
    step_ref = _to_integer(tensor)
    if step_ref is not None:
        if step_ref < 0 or step_ref >= step:
            raise ValueError(f"Out of order step reference {step_ref} for "
                             f"property {prop} at step {step}")
        geometry_metadata.property_references.add(
            geometry_property=plugin_data_pb2.Open3DPluginData.GeometryProperty.
            Value(prop),
            step_ref=step_ref)
        return None

    if prop.startswith("material_vector_"):
        save_tensor = _to_o3d(tensor, min_ndim=2, max_len=max_outputs)
        if save_tensor.ndim != 2 or save_tensor.shape[-1] != 4:
            raise ValueError(f"Material vector property {prop} has shape "
                             f"{tensor.shape} instead of (B, 4)")
        material["vector_properties"][prop[16:]] = save_tensor
        return save_tensor
    if prop.startswith("material_texture_map_"):
        save_tensor = _to_o3d(tensor, min_ndim=4, max_len=max_outputs)
        if save_tensor.ndim != 4:
            raise ValueError(f"Material texture map {prop} has shape "
                             f"{tensor.shape} instead of (B, Nr, Nc, C)")
        material["texture_maps"][prop[21:]] = tuple(
            o3d.t.geometry.Image(_color_to_uint8(save_tensor[bidx]))
            for bidx in range(save_tensor.shape[0]))
        return save_tensor
    if prop.startswith("material_"):
        raise ValueError(f"Unknown material property {prop}.")

    min_ndim = 4 if prop == "triangle_texture_uvs" else 3
    save_tensor = _to_o3d(tensor, min_ndim=min_ndim, max_len=max_outputs)
    if save_tensor.ndim != min_ndim:
        raise ValueError(f"Property {prop} tensor has shape {tensor.shape} "
                         f"instead of (B, N, 2) or (B, N, 3)")

    # Datatype conversion
    if prop.endswith("_colors"):
        save_tensor = _color_to_uint8(save_tensor)  # includes scaling
    elif prop.endswith("_indices"):
        save_tensor = save_tensor.to(dtype=o3d.core.int32)
    elif save_tensor.dtype == o3d.core.float64:
        save_tensor = save_tensor.to(dtype=o3d.core.float32)

    return save_tensor


def _write_geometry_data(write_dir, tag, step, data, max_outputs=3):
    """Serialize and write geometry data for a tag. Data is written to a
    separate file per tag.
    TODO: Add version specific reader / writer

    Args:
        write_dir (str): Path of folder to write data file.
        tag (str): Full name for geometry.
        step (int): Iteration / step count.
        data (dict): Property name to tensor mapping.
        max_outputs (int): Only the first `max_samples` data points in each
            batch will be saved.

    Returns:
        A comma separated data location string with the format
        f"{filename},{write_location},{write_size}"
    """

    if not isinstance(data, dict):
        raise TypeError(
            "data should be a dict of geometry property names and tensors.")
    unknown_props = [
        prop for prop in data if prop not in metadata.SUPPORTED_PROPERTIES
    ]
    if unknown_props:
        raise ValueError(
            f"Unknown geometry properties in data: {unknown_props}")
    if "vertex_positions" not in data:
        raise ValueError("Primary key 'vertex_positions' not provided.")
    if 'triangle_texture_uvs' in data and 'vertex_texture_uvs' in data:
        raise ValueError("Please provide only one of triangle_texture_uvs "
                         "or vertex_texture_uvs.")
    if max_outputs < 1:
        raise ValueError(
            f"max_outputs ({max_outputs}) should be a non-negative integer.")
    max_outputs = int(max_outputs)

    batch_size = None
    n_vertices = None
    n_triangles = None
    n_lines = None
    vertex_data = {}
    triangle_data = {}
    line_data = {}
    material = {
        "name": "",
        "scalar_properties": {},
        "vector_properties": {},
        "texture_maps": {}
    }

    def get_or_check_shape(prop, shape, exp_shape):
        for k, s in enumerate(exp_shape):
            if s is not None and s != shape[k]:
                raise ValueError(f"Property {prop} tensor should be of shape "
                                 f"{exp_shape} but is {shape}.")
        return shape[:2]

    geometry_metadata = plugin_data_pb2.Open3DPluginData(
        version=metadata._VERSION)
    o3d_type = "PointCloud"
    for prop, tensor in data.items():
        if prop in ('vertex_positions',) + metadata.VERTEX_PROPERTIES:
            prop_name = prop[7:]
            vertex_data[prop_name] = _preprocess(prop, tensor, step,
                                                 max_outputs, geometry_metadata)
            if vertex_data[prop_name] is None:  # Step reference
                del vertex_data[prop_name]
                continue
            # Get tensor dims from earlier property
            batch_size, n_vertices = get_or_check_shape(
                prop, vertex_data[prop_name].shape, (batch_size, n_vertices) +
                metadata.GEOMETRY_PROPERTY_DIMS[prop])

        elif prop in ('triangle_indices',) + metadata.TRIANGLE_PROPERTIES:
            o3d_type = "TriangleMesh"
            prop_name = prop[9:]
            triangle_data[prop_name] = _preprocess(prop, tensor, step,
                                                   max_outputs,
                                                   geometry_metadata)
            if triangle_data[prop_name] is None:  # Step reference
                del triangle_data[prop_name]
                continue
            # Get tensor dims from earlier property
            batch_size, n_triangles = get_or_check_shape(
                prop, triangle_data[prop_name].shape,
                (batch_size, n_triangles) +
                metadata.GEOMETRY_PROPERTY_DIMS[prop])

        elif prop in ('line_indices',) + metadata.LINE_PROPERTIES:
            if o3d_type != "TriangleMesh":
                o3d_type = "LineSet"
            prop_name = prop[5:]
            line_data[prop_name] = _preprocess(prop, tensor, step, max_outputs,
                                               geometry_metadata)
            if line_data[prop_name] is None:  # Step reference
                del line_data[prop_name]
                continue
            # Get tensor dims from earlier property
            batch_size, n_lines = get_or_check_shape(
                prop, line_data[prop_name].shape,
                (batch_size, n_lines) + metadata.GEOMETRY_PROPERTY_DIMS[prop])

        elif prop.startswith("material_"):  # Set property in material directly
            _preprocess(prop, tensor, step, max_outputs, geometry_metadata,
                        material)

    if (len(material["texture_maps"]) > 0 and
            'triangle_texture_uvs' not in data and
            'vertex_texture_uvs' not in data):
        raise ValueError("Please provide texture coordinates "
                         "'[vertex|triangle]_texture_uvs' to use texture maps.")
    vertices = vertex_data.pop("positions",
                               o3d.core.Tensor((), dtype=o3d.core.float32))
    faces = triangle_data.pop("indices",
                              o3d.core.Tensor((), dtype=o3d.core.int32))
    lines = line_data.pop("indices", o3d.core.Tensor((), dtype=o3d.core.int32))
    for bidx in range(batch_size):
        buf_con = o3d.io.rpc.BufferConnection()
        if not o3d.io.rpc.set_mesh_data(
                path=tag,
                time=step,
                layer="",
                vertices=vertices[bidx, :, :]
                if vertices.ndim == 3 else vertices,
                vertex_attributes={
                    prop: tensor[bidx, :, :]
                    for prop, tensor in vertex_data.items()
                },
                faces=faces[bidx, :, :] if faces.ndim == 3 else faces,
                face_attributes={
                    prop: tensor[bidx, :, :]
                    for prop, tensor in triangle_data.items()
                },
                lines=lines[bidx, :, :] if lines.ndim == 3 else lines,
                line_attributes={
                    prop: tensor[bidx, :, :]
                    for prop, tensor in line_data.items()
                },
                material=("" if material["name"] == "" else
                          material["name"][bidx]),
                material_scalar_attributes={
                    prop: val[bidx].item()
                    for prop, val in material["scalar_properties"].items()
                },
                material_vector_attributes={
                    prop: val[bidx].numpy()
                    for prop, val in material["vector_properties"].items()
                },
                texture_maps={
                    prop: val[bidx]
                    for prop, val in material["texture_maps"].items()
                },
                o3d_type=o3d_type,
                connection=buf_con):
            raise IOError(
                "[Open3D set_mesh_data] Geometry data serialization for tag "
                "{tag} step {step} failed!")
        # TODO(ssheorey): This returns a copy instead of the original. Benchmark
        # vs numpy
        data_buffer = buf_con.get_buffer()
        filename, this_write_location = _async_data_writer.enqueue(
            os.path.join(write_dir, tag.replace('/', '-')), data_buffer)
        if bidx == 0:
            geometry_metadata.batch_index.filename = filename
        geometry_metadata.batch_index.start_size.add(
            start=this_write_location,
            size=len(data_buffer),
            masked_crc32c=masked_crc32c(data_buffer))
    return geometry_metadata.SerializeToString()


def add_3d(name, data, step, logdir=None, max_outputs=1, description=None):
    """Write 3D geometry data as summary.

    Args:
      name (str): A name or tag for this summary. The summary tag used for TensorBoard
        will be this name prefixed by any active name scopes.
      data (dict): A dictionary of tensors representing 3D data. Tensorflow,
        PyTorch, Numpy and Open3D tensors are supported. The following keys
        are supported:
          - ``vertex_positions``: shape `(B, N, 3)` where B is the number of
                point clouds and must be same for each key. N is the number of
                3D points. Will be cast to ``float32``.
          - ``vertex_colors``: shape `(B, N, 3)` Will be converted to ``uint8``.
          - ``vertex_normals``: shape `(B, N, 3)` Will be cast to ``float32``.
          - ``vertex_texture_uvs``: shape `(B, N, 2)` Per vertex UV coordinates
            for applying material texture maps. Will be cast to ``float32``.
            Only one of ``vertex|triangle_texture_uvs`` should be provided.
          - ``triangle_indices``: shape `(B, Nf, 3)`. Will be cast to ``uint32``.
          - ``triangle_texture_uvs``: shape `(B, Nf, 3, 2)` Per triangle UV
            coordinates for applying material texture maps. Will be cast to
            ``float32``. Only one of ``[vertex|triangle]_texture_uvs`` should be
            provided.
          - ``line_indices``: shape `(B, Nl, 2)`. Will be cast to ``uint32``.
          - ``material_name``: shape `(B,)` and dtype ``str``. Base PBR material
            name is required to specify any material properties.  Open3D
            built-in materials: ``defaultLit``, ``defaultUnlit``, ``unlitLine``,
            ``unlitGradient``, ``unlitSolidColor``.
          - ``material_scalar_*PROPERTY*``: Any material scalar property with
            float values of shape `(B,)`. e.g. To specify the property
            `metallic`, use the key `material_scalar_metallic`.
          -  ``material_vector_*PROPERTY*``: Any material 4-vector property with
             float values of shape `(B, 4)` e.g. To specify the property
            `baseColor`, use the key `material_vector_base_color`.
          -  ``material_texture_map_*TEXTURE_MAP*``: PBR Material texture maps.
             e.g. ``material_texture_map_metallic`` represents a texture map
             describing the metallic property for rendering.
             Values are Tensors with shape `(B, Nr, Nc, C)`, corresponding to a
             batch of texture maps with `C` channels and shape `(Nr, Nc)`. The
             geometry must have ``[vertex|triangle]_texture_uvs`` coordinates to
             use any texture map.

        For batch_size B=1, the tensors may drop a rank (e.g. (N,3)
        vertex_positions, (4,) material vector properties or float scalar ()
        material scalar properties.)

        Floating point color and texture map data will be clipped to the range
        [0,1] and converted to ``uint8`` range [0,255]. ``uint16`` data will be
        compressed to the range [0,255].

        Any data tensor (with ndim>=2 including batch_size, i.e. excluding
        ``material_name`` and ``material_scalar_*PROPERTY*``), may be replaced by
        an ``int`` scalar referring to a previous step. This allows reusing a
        previously written property in case that it does not change at different
        steps.

        Please see the `Filament Materials Guide
        <https://google.github.io/filament/Materials.html#materialmodels>`__ for
        a complete explanation of material properties.

      step (int): Explicit ``int64``-castable monotonic step value for this summary.
        [`TensorFlow`: If ``None``, this defaults to
        `tf.summary.experimental.get_step()`, which must not be ``None``.]
      logdir (str): The logging directory used to create the SummaryWriter.
        [`PyTorch`: This will be automatically inferred if not provided or
        ``None``.]
      max_outputs (int): Optional integer. At most this many 3D elements will be
        emitted at each step. When more than `max_outputs` 3D elements are
        provided, the first ``max_outputs`` 3D elements will be used and the
        rest silently discarded.
      description (str): Optional long-form description for this summary.
        Markdown is supported. Defaults to empty. Currently unused.

    Returns:
      True on success, or false if no summary was emitted because no default
      summary writer was available.

    Raises:
      ValueError: if a default writer exists, but no step was provided and
        `tf.summary.experimental.get_step()` is None. Also raised when used with
        Tensorflow and ``logdir`` is not provided or ``None``.
      RuntimeError: Module level function is used without a TensorFlow
        installation. Use the PyTorch `SummaryWriter.add_3d()` bound method
        instead.

    Examples:
        # TODO(ssheorey): Update example with material properties.
        With Tensorflow:

        .. code::

            import tensorflow as tf
            import open3d as o3d
            from open3d.visualization.tensorboard_plugin import summary
            from open3d.visualization.tensorboard_plugin.util import to_dict_batch

            logdir = "demo_logs/"
            writer = tf.summary.create_file_writer(logdir)
            cube = o3d.geometry.TriangleMesh.create_box(1, 2, 4,
                create_uv_map=True)
            cube.compute_vertex_normals()
            colors = [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)]
            with writer.as_default():
                for step in range(3):
                    cube.paint_uniform_color(colors[step])
                    summary.add_3d('cube',
                                   to_dict_batch([cube]),
                                   step=step,
                                   logdir=logdir)

        With PyTorch:

        .. code::

            from torch.utils.tensorboard import SummaryWriter
            import open3d as o3d
            from open3d.visualization.tensorboard_plugin import summary
            from open3d.visualization.tensorboard_plugin.util import to_dict_batch
            writer = SummaryWriter("demo_logs/")
            cube = o3d.geometry.TriangleMesh.create_box(1, 2, 4,
                create_uv_map=True)
            cube.compute_vertex_normals()
            colors = [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)]
            for step in range(3):
                cube.paint_uniform_color(colors[step])
                writer.add_3d('cube', to_dict_batch([cube]), step=step)

        Now use ``tensorboard --logdir demo_logs`` to visualize the 3D data.
    """
    if tf is None:
        raise RuntimeError(
            "TensorFlow not found. Please use module level ``add_3d`` only "
            "with TensorFlow. Use the bound method ``SummaryWriter.add_3d`` "
            "with PyTorch.")
    if step is None:
        step = tf.summary.experimental.get_step()
    if step is None:
        raise ValueError("Step is not provided or set.")
    if logdir is None:
        raise ValueError("logdir must be provided with TensorFlow.")

    summary_metadata = metadata.create_summary_metadata(description=description)
    # TODO(https://github.com/tensorflow/tensorboard/issues/2109): remove fallback
    summary_scope = (getattr(tf.summary.experimental, "summary_scope", None) or
                     tf.summary.summary_scope)
    with summary_scope(name, "open3d_summary",
                       values=[data, max_outputs, step]) as (tag, unused_scope):
        # Defer preprocessing by passing it as a callable to write(),
        # wrapped in a LazyTensorCreator for backwards compatibility, so that we
        # only do this work when summaries are actually written, i.e. if
        # record_if() returns True.
        @lazy_tensor_creator.LazyTensorCreator
        def lazy_tensor():
            write_dir = PluginDirectory(logdir, metadata.PLUGIN_NAME)
            geometry_metadata_string = _write_geometry_data(
                write_dir, tag, step, data, max_outputs)
            return tf.convert_to_tensor(geometry_metadata_string)

        return tf.summary.write(tag=tag,
                                tensor=lazy_tensor,
                                step=step,
                                metadata=summary_metadata)


def _add_3d_torch(self,
                  tag,
                  data,
                  step,
                  logdir=None,
                  max_outputs=1,
                  description=None):
    walltime = None
    if step is None:
        raise ValueError("Step is not provided or set.")
    summary_metadata = metadata.create_summary_metadata(description=description)
    if logdir is None:
        logdir = self._get_file_writer().get_logdir()
    write_dir = PluginDirectory(logdir, metadata.PLUGIN_NAME)
    geometry_metadata_string = _write_geometry_data(write_dir, tag, step, data,
                                                    max_outputs)
    tensor_proto = TensorProto(dtype='DT_STRING',
                               string_val=[geometry_metadata_string],
                               tensor_shape=TensorShapeProto())

    self._get_file_writer().add_summary(
        Summary(value=[
            Summary.Value(
                tag=tag, tensor=tensor_proto, metadata=summary_metadata)
        ]), step, walltime)


# Make _add_3d_torch a bound method of SummaryWriter class. (MonkeyPatching)
if torch is not None:
    if not hasattr(SummaryWriter, "add_3d"):
        SummaryWriter.add_3d = _add_3d_torch
        SummaryWriter.add_3d.__doc__ = add_3d.__doc__  # Copy docstring from TF function
    else:
        warnings.warn("Cannot bind add_3d() to SummaryWriter. Binding exists.",
                      RuntimeWarning)
