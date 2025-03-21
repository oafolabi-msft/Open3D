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
# ----------------------------------------------------------------------------
import copy
import os
import sys
import numpy as np
import open3d as o3d
from open3d.visualization.tensorboard_plugin import summary
from open3d.visualization.tensorboard_plugin.util import to_dict_batch
import tensorflow as tf

BASE_LOGDIR = "demo_logs/tf/"
MODEL_DIR = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..',
                         '..', "test_data", "monkey")


def small_scale(run_name="small_scale"):
    """Basic demo with cube and cylinder with normals and colors.
    """
    logdir = os.path.join(BASE_LOGDIR, run_name)
    writer = tf.summary.create_file_writer(logdir)

    cube = o3d.geometry.TriangleMesh.create_box(1, 2, 4, create_uv_map=True)
    cube.compute_vertex_normals()
    cylinder = o3d.geometry.TriangleMesh.create_cylinder(radius=1.0,
                                                         height=2.0,
                                                         resolution=20,
                                                         split=4,
                                                         create_uv_map=True)
    cylinder.compute_vertex_normals()
    colors = [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)]
    with writer.as_default():
        for step in range(3):
            cube.paint_uniform_color(colors[step])
            summary.add_3d('cube',
                           to_dict_batch([cube]),
                           step=step,
                           logdir=logdir)
            cylinder.paint_uniform_color(colors[step])
            summary.add_3d('cylinder',
                           to_dict_batch([cylinder]),
                           step=step,
                           logdir=logdir)


def property_reference(run_name="property_reference"):
    """Produces identical visualization to small_scale, but does not store
    repeated properties of ``vertex_positions`` and ``vertex_normals``.
    """
    logdir = os.path.join(BASE_LOGDIR, run_name)
    writer = tf.summary.create_file_writer(logdir)

    cube = o3d.geometry.TriangleMesh.create_box(1, 2, 4, create_uv_map=True)
    cube.compute_vertex_normals()
    cylinder = o3d.geometry.TriangleMesh.create_cylinder(radius=1.0,
                                                         height=2.0,
                                                         resolution=20,
                                                         split=4,
                                                         create_uv_map=True)
    cylinder.compute_vertex_normals()
    colors = [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)]
    with writer.as_default():
        for step in range(3):
            cube.paint_uniform_color(colors[step])
            cube_summary = to_dict_batch([cube])
            if step > 0:
                cube_summary['vertex_positions'] = 0
                cube_summary['vertex_normals'] = 0
            summary.add_3d('cube', cube_summary, step=step, logdir=logdir)
            cylinder.paint_uniform_color(colors[step])
            cylinder_summary = to_dict_batch([cylinder])
            if step > 0:
                cylinder_summary['vertex_positions'] = 0
                cylinder_summary['vertex_normals'] = 0
            summary.add_3d('cylinder',
                           cylinder_summary,
                           step=step,
                           logdir=logdir)


def large_scale(n_steps=16,
                batch_size=1,
                base_resolution=200,
                run_name="large_scale"):
    """Generate a large scale summary. Geometry resolution increases linearly
    with step. Each element in a batch is painted a different color.
    """
    logdir = os.path.join(BASE_LOGDIR, run_name)
    writer = tf.summary.create_file_writer(logdir)
    colors = []
    for k in range(batch_size):
        t = k * np.pi / batch_size
        colors.append(((1 + np.sin(t)) / 2, (1 + np.cos(t)) / 2, t / np.pi))
    with writer.as_default():
        for step in range(n_steps):
            resolution = base_resolution * (step + 1)
            cylinder_list = []
            moebius_list = []
            cylinder = o3d.geometry.TriangleMesh.create_cylinder(
                radius=1.0, height=2.0, resolution=resolution, split=4)
            cylinder.compute_vertex_normals()
            moebius = o3d.geometry.TriangleMesh.create_moebius(
                length_split=int(3.5 * resolution),
                width_split=int(0.75 * resolution),
                twists=1,
                raidus=1,
                flatness=1,
                width=1,
                scale=1)
            moebius.compute_vertex_normals()
            for b in range(batch_size):
                cylinder_list.append(copy.deepcopy(cylinder))
                cylinder_list[b].paint_uniform_color(colors[b])
                moebius_list.append(copy.deepcopy(moebius))
                moebius_list[b].paint_uniform_color(colors[b])
            summary.add_3d('cylinder',
                           to_dict_batch(cylinder_list),
                           step=step,
                           logdir=logdir,
                           max_outputs=batch_size)
            summary.add_3d('moebius',
                           to_dict_batch(moebius_list),
                           step=step,
                           logdir=logdir,
                           max_outputs=batch_size)


def with_material(model_dir=MODEL_DIR):
    """Read an obj model from a directory and write as a TensorBoard summary.
    """
    model_name = os.path.basename(model_dir)
    logdir = os.path.join(BASE_LOGDIR, model_name)
    model_path = os.path.join(model_dir, model_name + ".obj")
    model = o3d.t.geometry.TriangleMesh.from_legacy(
        o3d.io.read_triangle_mesh(model_path))
    summary_3d = {
        "vertex_positions": model.vertex["positions"],
        "vertex_normals": model.vertex["normals"],
        "triangle_texture_uvs": model.triangle["texture_uvs"],
        "triangle_indices": model.triangle["indices"],
        "material_name": "defaultLit"
    }
    names_to_o3dprop = {"ao": "ambient_occlusion"}

    for texture in ("albedo", "normal", "ao", "metallic", "roughness"):
        texture_file = os.path.join(model_dir, texture + ".png")
        if os.path.exists(texture_file):
            texture = names_to_o3dprop.get(texture, texture)
            summary_3d.update({
                ("material_texture_map_" + texture):
                    o3d.t.io.read_image(texture_file)
            })
            if texture == "metallic":
                summary_3d.update(material_scalar_metallic=1.0)

    writer = tf.summary.create_file_writer(logdir)
    with writer.as_default():
        summary.add_3d(model_name, summary_3d, step=0, logdir=logdir)


if __name__ == "__main__":

    examples = ('small_scale', 'large_scale', 'property_reference',
                'with_material')
    selected = tuple(eg for eg in sys.argv[1:] if eg in examples)
    if len(selected) == 0:
        print(f'Usage: python {__file__} EXAMPLE...')
        print(f'  where EXAMPLE are from {examples}')
        selected = ('property_reference', 'with_material')

    for eg in selected:
        locals()[eg]()

    print(f"Run 'tensorboard --logdir {BASE_LOGDIR}' to visualize the 3D "
          "summary.")
