//
// Implementation for Yocto/RayTrace.
//

//
// LICENSE:
//
// Copyright (c) 2016 -- 2020 Fabio Pellacini
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include "yocto_pathtrace.h"

#include <yocto/yocto_color.h>
#include <yocto/yocto_geometry.h>
#include <yocto/yocto_parallel.h>
#include <yocto/yocto_sampling.h>
#include <yocto/yocto_shading.h>
#include <yocto/yocto_shape.h>

#include <chrono>

// -----------------------------------------------------------------------------
// IMPLEMENTATION FOR SCENE EVALUATION
// -----------------------------------------------------------------------------
namespace yocto {

// Check texture size
static vec2i texture_size(const pathtrace_texture* texture) {
  if (!texture->hdr.empty()) {
    return texture->hdr.imsize();
  } else if (!texture->ldr.empty()) {
    return texture->ldr.imsize();
  } else {
    return zero2i;
  }
}

// Evaluate a texture
static vec4f lookup_texture(const pathtrace_texture* texture, const vec2i& ij,
    bool ldr_as_linear = false) {
  if (!texture->hdr.empty()) {
    return texture->hdr[ij];
  } else if (!texture->ldr.empty()) {
    return ldr_as_linear ? byte_to_float(texture->ldr[ij])
                         : srgb_to_rgb(byte_to_float(texture->ldr[ij]));
  } else {
    return {1, 1, 1};
  }
}

// Evaluate a texture
static vec4f eval_texture(const pathtrace_texture* texture, const vec2f& uv,
    bool ldr_as_linear = false, bool no_interpolation = false,
    bool clamp_to_edge = false) {
  // get texture
  if (!texture) return {1, 1, 1};

  // get yimage width/height
  auto size = texture_size(texture);

  // get coordinates normalized for tiling
  auto s = 0.0f, t = 0.0f;
  if (clamp_to_edge) {
    s = clamp(uv.x, 0.0f, 1.0f) * size.x;
    t = clamp(uv.y, 0.0f, 1.0f) * size.y;
  } else {
    s = fmod(uv.x, 1.0f) * size.x;
    if (s < 0) s += size.x;
    t = fmod(uv.y, 1.0f) * size.y;
    if (t < 0) t += size.y;
  }

  // get yimage coordinates and residuals
  auto i = clamp((int)s, 0, size.x - 1), j = clamp((int)t, 0, size.y - 1);
  auto ii = (i + 1) % size.x, jj = (j + 1) % size.y;
  auto u = s - i, v = t - j;

  if (no_interpolation) return lookup_texture(texture, {i, j}, ldr_as_linear);

  // handle interpolation
  return lookup_texture(texture, {i, j}, ldr_as_linear) * (1 - u) * (1 - v) +
         lookup_texture(texture, {i, jj}, ldr_as_linear) * (1 - u) * v +
         lookup_texture(texture, {ii, j}, ldr_as_linear) * u * (1 - v) +
         lookup_texture(texture, {ii, jj}, ldr_as_linear) * u * v;
}

// Generates a ray from a camera for yimage plane coordinate uv and
// the lens coordinates luv.
static ray3f eval_camera(const pathtrace_camera* camera, const vec2f& image_uv,
    const vec2f& lens_uv) {
  auto q  = vec3f{camera->film.x * (0.5f - image_uv.x),
      camera->film.y * (image_uv.y - 0.5f), camera->lens};
  auto dc = -normalize(q);
  auto e  = vec3f{
      lens_uv.x * camera->aperture / 2, lens_uv.y * camera->aperture / 2, 0};
  auto p = dc * camera->focus / abs(dc.z);
  auto d = normalize(p - e);
  return ray3f{
      transform_point(camera->frame, e), transform_direction(camera->frame, d)};
}

// Samples a camera ray at pixel ij of an image of size size with puv and luv
// as random numbers for pixel and lens respectively
static ray3f sample_camera(const pathtrace_camera* camera, const vec2i& ij,
    const vec2i& size, const vec2f& puv, const vec2f& luv) {
  return eval_camera(camera,
      vec2f{(ij.x + puv.x) / size.x, (ij.y + puv.y) / size.y},
      sample_disk(luv));
}

// Eval position
static vec3f eval_position(
    const pathtrace_instance* instance, int element, const vec2f& uv) {
  auto shape = instance->shape;
  if (!shape->triangles.empty()) {
    auto t = shape->triangles[element];
    return transform_point(
        instance->frame, interpolate_triangle(shape->positions[t.x],
                             shape->positions[t.y], shape->positions[t.z], uv));
  } else if (!shape->quads.empty()) {
    auto q = shape->quads[element];
    return transform_point(instance->frame,
        interpolate_quad(shape->positions[q.x], shape->positions[q.y],
            shape->positions[q.z], shape->positions[q.w], uv));
  } else if (!shape->lines.empty()) {
    auto l = shape->lines[element];
    return transform_point(instance->frame,
        interpolate_line(shape->positions[l.x], shape->positions[l.y], uv.x));
  } else if (!shape->points.empty()) {
    return transform_point(
        instance->frame, shape->positions[shape->points[element]]);
  } else {
    return zero3f;
  }
}

// Shape element normal.
static vec3f eval_element_normal(
    const pathtrace_instance* instance, int element) {
  auto shape = instance->shape;
  if (!shape->triangles.empty()) {
    auto t = shape->triangles[element];
    return transform_normal(
        instance->frame, triangle_normal(shape->positions[t.x],
                             shape->positions[t.y], shape->positions[t.z]));
  } else if (!shape->quads.empty()) {
    auto q = shape->quads[element];
    return transform_normal(instance->frame,
        quad_normal(shape->positions[q.x], shape->positions[q.y],
            shape->positions[q.z], shape->positions[q.w]));
  } else if (!shape->lines.empty()) {
    auto l = shape->lines[element];
    return transform_normal(instance->frame,
        line_tangent(shape->positions[l.x], shape->positions[l.y]));
  } else if (!shape->points.empty()) {
    return {0, 0, 1};
  } else {
    return {0, 0, 0};
  }
}

// Eval normal
static vec3f eval_normal(
    const pathtrace_instance* instance, int element, const vec2f& uv) {
  auto shape = instance->shape;
  if (shape->normals.empty()) return eval_element_normal(instance, element);
  if (!shape->triangles.empty()) {
    auto t = shape->triangles[element];
    return transform_normal(
        instance->frame, normalize(interpolate_triangle(shape->normals[t.x],
                             shape->normals[t.y], shape->normals[t.z], uv)));
  } else if (!shape->quads.empty()) {
    auto q = shape->quads[element];
    return transform_normal(instance->frame,
        normalize(interpolate_quad(shape->normals[q.x], shape->normals[q.y],
            shape->normals[q.z], shape->normals[q.w], uv)));
  } else if (!shape->lines.empty()) {
    auto l = shape->lines[element];
    return transform_normal(instance->frame,
        normalize(
            interpolate_line(shape->normals[l.x], shape->normals[l.y], uv.x)));
  } else if (!shape->points.empty()) {
    return transform_normal(
        instance->frame, normalize(shape->normals[shape->points[element]]));
  } else {
    return zero3f;
  }
}

// Eval texcoord
static vec2f eval_texcoord(
    const pathtrace_instance* instance, int element, const vec2f& uv) {
  auto shape = instance->shape;
  if (shape->texcoords.empty()) return uv;
  if (!shape->triangles.empty()) {
    auto t = shape->triangles[element];
    return interpolate_triangle(shape->texcoords[t.x], shape->texcoords[t.y],
        shape->texcoords[t.z], uv);
  } else if (!shape->quads.empty()) {
    auto q = shape->quads[element];
    return interpolate_quad(shape->texcoords[q.x], shape->texcoords[q.y],
        shape->texcoords[q.z], shape->texcoords[q.w], uv);
  } else if (!shape->lines.empty()) {
    auto l = shape->lines[element];
    return interpolate_line(shape->texcoords[l.x], shape->texcoords[l.y], uv.x);
  } else if (!shape->points.empty()) {
    return shape->texcoords[shape->points[element]];
  } else {
    return zero2f;
  }
}

// Shape element normal.
static pair<vec3f, vec3f> eval_element_tangents(
    const pathtrace_instance* instance, int element) {
  auto shape = instance->shape;
  if (!shape->triangles.empty() && !shape->texcoords.empty()) {
    auto t        = shape->triangles[element];
    auto [tu, tv] = triangle_tangents_fromuv(shape->positions[t.x],
        shape->positions[t.y], shape->positions[t.z], shape->texcoords[t.x],
        shape->texcoords[t.y], shape->texcoords[t.z]);
    return {transform_direction(instance->frame, tu),
        transform_direction(instance->frame, tv)};
  } else if (!shape->quads.empty() && !shape->texcoords.empty()) {
    auto q        = shape->quads[element];
    auto [tu, tv] = quad_tangents_fromuv(shape->positions[q.x],
        shape->positions[q.y], shape->positions[q.z], shape->positions[q.w],
        shape->texcoords[q.x], shape->texcoords[q.y], shape->texcoords[q.z],
        shape->texcoords[q.w], {0, 0});
    return {transform_direction(instance->frame, tu),
        transform_direction(instance->frame, tv)};
  } else {
    return {};
  }
}

// YOUR CODE GOES HERE -----------------------------------------------------
static vec3f eval_normalmap(
    const pathtrace_instance* instance, int element, const vec2f& uv) {
  auto shape      = instance->shape;
  auto normal_tex = instance->material->normal_tex;
  auto normal     = eval_normal(instance, element, uv);
  auto texcoord   = eval_texcoord(instance, element, uv);
  if (normal_tex != nullptr &&
      (!shape->triangles.empty() || !shape->quads.empty())) {
    auto nmap     = -1 + 2 * xyz(eval_texture(normal_tex, texcoord, true));
    auto [tu, tv] = eval_element_tangents(instance, element);
    auto frame    = frame3f{tu, tv, normal, zero3f};
    frame.x       = orthonormalize(frame.x, frame.z);
    frame.y       = normalize(cross(frame.z, frame.x));
    nmap.y *= dot(frame.y, tv) < 0 ? 1 : -1;
    normal = transform_normal(frame, nmap);
  }
  return normal;
}

// Eval shading normal
static vec3f eval_shading_normal(const pathtrace_instance* instance,
    int element, const vec2f& uv, const vec3f& outgoing) {
  auto shape  = instance->shape;
  auto normal = eval_normal(instance, element, uv);
  if (!shape->triangles.empty() || !shape->quads.empty()) {
    if (instance->material->normal_tex != nullptr) {
      normal = eval_normalmap(instance, element, uv);
    }
    if (!instance->material->thin) return normal;
    return dot(normal, outgoing) >= 0 ? normal : -normal;
  } else if (!shape->lines.empty()) {
    return orthonormalize(outgoing, normal);
  } else if (!shape->points.empty()) {
    return -outgoing;
  } else {
    return zero3f;
  }
}

// Brdf
struct pathtrace_brdf {
  // brdf lobes
  vec3f diffuse      = {0, 0, 0};
  vec3f specular     = {0, 0, 0};
  vec3f metal        = {0, 0, 0};
  vec3f transmission = {0, 0, 0};
  vec3f refraction   = {0, 0, 0};
  float roughness    = 0;
  float opacity      = 1;
  float ior          = 1;
  vec3f meta         = {0, 0, 0};
  vec3f metak        = {0, 0, 0};
  // weights
  float diffuse_pdf      = 0;
  float specular_pdf     = 0;
  float metal_pdf        = 0;
  float transmission_pdf = 0;
  float refraction_pdf   = 0;
};

// Eval material to obatain emission, brdf and opacity.
static vec3f eval_emission(const pathtrace_instance* instance, int element,
    const vec2f& uv, const vec3f& normal, const vec3f& outgoing) {
  auto material = instance->material;
  auto texcoord = eval_texcoord(instance, element, uv);
  return material->emission *
         xyz(eval_texture(material->emission_tex, texcoord));
}

// Eval material to obatain emission, brdf and opacity.
static pathtrace_brdf eval_brdf(const pathtrace_instance* instance, int element,
    const vec2f& uv, const vec3f& normal, const vec3f& outgoing) {
  auto material = instance->material;
  auto texcoord = eval_texcoord(instance, element, uv);
  auto brdf     = pathtrace_brdf{};
  auto color    = material->color *
               xyz(eval_texture(material->color_tex, texcoord, false));
  auto specular = material->specular *
                  eval_texture(material->specular_tex, texcoord, true).x;
  auto metallic = material->metallic *
                  eval_texture(material->metallic_tex, texcoord, true).x;
  auto roughness = material->roughness *
                   eval_texture(material->roughness_tex, texcoord, true).x;
  auto ior          = material->ior;
  auto transmission = material->transmission *
                      eval_texture(material->emission_tex, texcoord, true).x;
  auto opacity = material->opacity *
                 eval_texture(material->opacity_tex, texcoord, true).x;
  auto thin = material->thin || material->transmission == 0;

  // factors
  auto weight = vec3f{1, 1, 1};
  brdf.metal  = weight * metallic;
  weight *= 1 - metallic;
  brdf.refraction = thin ? zero3f : weight * transmission;
  weight *= 1 - (thin ? 0 : transmission);
  brdf.specular = weight * specular;
  weight *= 1 - specular * fresnel_dielectric(ior, outgoing, normal);
  brdf.transmission = weight * transmission * color;
  weight *= 1 - transmission;
  brdf.diffuse   = weight * color;
  brdf.meta      = reflectivity_to_eta(color);
  brdf.metak     = zero3f;
  brdf.roughness = roughness * roughness;
  brdf.ior       = ior;
  brdf.opacity   = opacity;

  // textures
  if (brdf.diffuse != zero3f || brdf.roughness != 0) {
    brdf.roughness = clamp(brdf.roughness, 0.03f * 0.03f, 1.0f);
  }
  if (brdf.specular == zero3f && brdf.metal == zero3f &&
      brdf.transmission == zero3f && brdf.refraction == zero3f) {
    brdf.roughness = 1;
  }
  if (brdf.opacity > 0.999f) brdf.opacity = 1;

  // weights
  brdf.diffuse_pdf  = max(brdf.diffuse);
  brdf.specular_pdf = max(
      brdf.specular * fresnel_dielectric(brdf.ior, normal, outgoing));
  brdf.metal_pdf = max(
      brdf.metal * fresnel_conductor(brdf.meta, brdf.metak, normal, outgoing));
  brdf.transmission_pdf = max(brdf.transmission);
  brdf.refraction_pdf   = max(brdf.refraction);
  auto pdf_sum = brdf.diffuse_pdf + brdf.specular_pdf + brdf.metal_pdf +
                 brdf.transmission_pdf + brdf.refraction_pdf;
  if (pdf_sum != 0) {
    brdf.diffuse_pdf /= pdf_sum;
    brdf.specular_pdf /= pdf_sum;
    brdf.metal_pdf /= pdf_sum;
    brdf.transmission_pdf /= pdf_sum;
    brdf.refraction_pdf /= pdf_sum;
  }
  return brdf;
}

// check if a brdf is a delta
static bool is_delta(const pathtrace_brdf& brdf) { return brdf.roughness == 0; }

// vsdf
struct pathtrace_vsdf {
  vec3f density    = {0, 0, 0};
  vec3f scatter    = {0, 0, 0};
  float anisotropy = 0;
};

// evaluate volume
static pathtrace_vsdf eval_vsdf(
    const pathtrace_instance* instance, int element, const vec2f& uv) {
  auto material = instance->material;
  // initialize factors
  auto texcoord = eval_texcoord(instance, element, uv);
  auto base     = material->color *
              xyz(eval_texture(material->color_tex, texcoord, false));
  auto transmission = material->transmission *
                      eval_texture(material->emission_tex, texcoord, true).x;
  auto thin = material->thin || material->transmission == 0;
  auto scattering =
      material->scattering *
      xyz(eval_texture(material->scattering_tex, texcoord, false));
  auto scanisotropy = material->scanisotropy;
  auto trdepth      = material->trdepth;

  // factors
  auto vsdf    = pathtrace_vsdf{};
  vsdf.density = (transmission != 0 && !thin)
                     ? -log(clamp(base, 0.0001f, 1.0f)) / trdepth
                     : zero3f;
  vsdf.scatter    = scattering;
  vsdf.anisotropy = scanisotropy;

  return vsdf;
}

// check if we have a volume
static bool has_volume(const pathtrace_instance* instance) {
  return !instance->material->thin && instance->material->transmission != 0;
}

// Evaluate all environment color.
static vec3f eval_environment(const pathtrace_scene* scene, const ray3f& ray) {
  auto emission = zero3f;
  for (auto environment : scene->environments) {
    auto wl       = transform_direction(inverse(environment->frame), ray.d);
    auto texcoord = vec2f{
        atan2(wl.z, wl.x) / (2 * pif), acos(clamp(wl.y, -1.0f, 1.0f)) / pif};
    if (texcoord.x < 0) texcoord.x += 1;
    emission += environment->emission *
                xyz(eval_texture(environment->emission_tex, texcoord));
  }
  return emission;
}

}  // namespace yocto

// -----------------------------------------------------------------------------
// IMPLEMENTATION FOR SHAPE/SCENE BVH
// -----------------------------------------------------------------------------
namespace yocto {

// primitive used to sort bvh entries
struct pathtrace_bvh_primitive {
  bbox3f bbox      = invalidb3f;
  vec3f  center    = zero3f;
  int    primitive = 0;
};

// Splits a BVH node. Returns split position and axis.
static pair<int, int> split_middle(
    vector<pathtrace_bvh_primitive>& primitives, int start, int end) {
  // initialize split axis and position
  auto axis = 0;
  auto mid  = (start + end) / 2;

  // compute primintive bounds and size
  auto cbbox = invalidb3f;
  for (auto i = start; i < end; i++) cbbox = merge(cbbox, primitives[i].center);
  auto csize = cbbox.max - cbbox.min;
  if (csize == zero3f) return {mid, axis};

  // split along largest
  if (csize.x >= csize.y && csize.x >= csize.z) axis = 0;
  if (csize.y >= csize.x && csize.y >= csize.z) axis = 1;
  if (csize.z >= csize.x && csize.z >= csize.y) axis = 2;

  // split the space in the middle along the largest axis
  mid = (int)(std::partition(primitives.data() + start, primitives.data() + end,
                  [axis, middle = center(cbbox)[axis]](auto& primitive) {
                    return primitive.center[axis] < middle;
                  }) -
              primitives.data());

  // if we were not able to split, just break the primitives in half
  if (mid == start || mid == end) {
    // throw runtime_error("bad bvh split");
    mid = (start + end) / 2;
  }

  return {mid, axis};
}

// Maximum number of primitives per BVH node.
const int bvh_max_prims = 4;

// Build BVH nodes
static void build_bvh(vector<pathtrace_bvh_node>& nodes,
    vector<pathtrace_bvh_primitive>&              primitives) {
  // prepare to build nodes
  nodes.clear();
  nodes.reserve(primitives.size() * 2);

  // queue up first node
  auto queue = deque<vec3i>{{0, 0, (int)primitives.size()}};
  nodes.emplace_back();

  // create nodes until the queue is empty
  while (!queue.empty()) {
    // grab node to work on
    auto next = queue.front();
    queue.pop_front();
    auto nodeid = next.x, start = next.y, end = next.z;

    // grab node
    auto& node = nodes[nodeid];

    // compute bounds
    node.bbox = invalidb3f;
    for (auto i = start; i < end; i++)
      node.bbox = merge(node.bbox, primitives[i].bbox);

    // split into two children
    if (end - start > bvh_max_prims) {
      // get split
      auto [mid, axis] = split_middle(primitives, start, end);

      // make an internal node
      node.internal = true;
      node.axis     = (byte)axis;
      node.num      = 2;
      node.start    = (int)nodes.size();
      nodes.emplace_back();
      nodes.emplace_back();
      queue.push_back({node.start + 0, start, mid});
      queue.push_back({node.start + 1, mid, end});
    } else {
      // Make a leaf node
      node.internal = false;
      node.num      = (short)(end - start);
      node.start    = start;
    }
  }

  // cleanup
  nodes.shrink_to_fit();
}

static void init_bvh(pathtrace_shape* shape, const pathtrace_params& params) {
  // build primitives
  auto primitives = vector<pathtrace_bvh_primitive>{};
  if (!shape->points.empty()) {
    for (auto idx = 0; idx < shape->points.size(); idx++) {
      auto& p             = shape->points[idx];
      auto& primitive     = primitives.emplace_back();
      primitive.bbox      = point_bounds(shape->positions[p], shape->radius[p]);
      primitive.center    = center(primitive.bbox);
      primitive.primitive = idx;
    }
  } else if (!shape->lines.empty()) {
    for (auto idx = 0; idx < shape->lines.size(); idx++) {
      auto& l         = shape->lines[idx];
      auto& primitive = primitives.emplace_back();
      primitive.bbox = line_bounds(shape->positions[l.x], shape->positions[l.y],
          shape->radius[l.x], shape->radius[l.y]);
      primitive.center    = center(primitive.bbox);
      primitive.primitive = idx;
    }
  } else if (!shape->triangles.empty()) {
    for (auto idx = 0; idx < shape->triangles.size(); idx++) {
      auto& primitive = primitives.emplace_back();
      auto& t         = shape->triangles[idx];
      primitive.bbox  = triangle_bounds(
          shape->positions[t.x], shape->positions[t.y], shape->positions[t.z]);
      primitive.center    = center(primitive.bbox);
      primitive.primitive = idx;
    }
  } else if (!shape->quads.empty()) {
    for (auto idx = 0; idx < shape->quads.size(); idx++) {
      auto& primitive = primitives.emplace_back();
      auto& q         = shape->quads[idx];
      primitive.bbox = quad_bounds(shape->positions[q.x], shape->positions[q.y],
          shape->positions[q.z], shape->positions[q.w]);
      primitive.center    = center(primitive.bbox);
      primitive.primitive = idx;
    }
  }

  // build nodes
  if (shape->bvh) delete shape->bvh;
  shape->bvh = new pathtrace_bvh_tree{};
  build_bvh(shape->bvh->nodes, primitives);

  // set bvh primitives
  shape->bvh->primitives.reserve(primitives.size());
  for (auto& primitive : primitives) {
    shape->bvh->primitives.push_back(primitive.primitive);
  }
}

void init_bvh(pathtrace_scene* scene, const pathtrace_params& params,
    progress_callback progress_cb) {
  // handle progress
  auto progress = vec2i{0, 1 + (int)scene->shapes.size()};

  // shapes
  for (auto idx = 0; idx < scene->shapes.size(); idx++) {
    if (progress_cb) progress_cb("build shape bvh", progress.x++, progress.y);
    init_bvh(scene->shapes[idx], params);
  }

  // handle progress
  if (progress_cb) progress_cb("build scene bvh", progress.x++, progress.y);

  // instance bboxes
  auto primitives  = vector<pathtrace_bvh_primitive>{};
  auto instance_id = 0;
  for (auto instance : scene->instances) {
    auto& primitive = primitives.emplace_back();
    primitive.bbox  = instance->shape->bvh->nodes.empty()
                         ? invalidb3f
                         : transform_bbox(instance->frame,
                               instance->shape->bvh->nodes[0].bbox);
    primitive.center    = center(primitive.bbox);
    primitive.primitive = instance_id++;
  }

  // build nodes
  if (scene->bvh) delete scene->bvh;
  scene->bvh = new pathtrace_bvh_tree{};
  build_bvh(scene->bvh->nodes, primitives);

  // set bvh primitives
  scene->bvh->primitives.reserve(primitives.size());
  for (auto& primitive : primitives) {
    scene->bvh->primitives.push_back(primitive.primitive);
  }

  // handle progress
  if (progress_cb) progress_cb("build bvh", progress.x++, progress.y);
}

// Intersect ray with a bvh->
static bool intersect_shape_bvh(pathtrace_shape* shape, const ray3f& ray_,
    int& element, vec2f& uv, float& distance, bool find_any) {
  // get bvh and shape pointers for fast access
  auto bvh = shape->bvh;

  // check empty
  if (bvh->nodes.empty()) return false;

  // node stack
  int  node_stack[128];
  auto node_cur          = 0;
  node_stack[node_cur++] = 0;

  // shared variables
  auto hit = false;

  // copy ray to modify it
  auto ray = ray_;

  // prepare ray for fast queries
  auto ray_dinv  = vec3f{1 / ray.d.x, 1 / ray.d.y, 1 / ray.d.z};
  auto ray_dsign = vec3i{(ray_dinv.x < 0) ? 1 : 0, (ray_dinv.y < 0) ? 1 : 0,
      (ray_dinv.z < 0) ? 1 : 0};

  // walking stack
  while (node_cur) {
    // grab node
    auto& node = bvh->nodes[node_stack[--node_cur]];

    // intersect bbox
    // if (!intersect_bbox(ray, ray_dinv, ray_dsign, node.bbox)) continue;
    if (!intersect_bbox(ray, ray_dinv, node.bbox)) continue;

    // intersect node, switching based on node type
    // for each type, iterate over the the primitive list
    if (node.internal) {
      // for internal nodes, attempts to proceed along the
      // split axis from smallest to largest nodes
      if (ray_dsign[node.axis]) {
        node_stack[node_cur++] = node.start + 0;
        node_stack[node_cur++] = node.start + 1;
      } else {
        node_stack[node_cur++] = node.start + 1;
        node_stack[node_cur++] = node.start + 0;
      }
    } else if (!shape->points.empty()) {
      for (auto idx = node.start; idx < node.start + node.num; idx++) {
        auto& p = shape->points[shape->bvh->primitives[idx]];
        if (intersect_point(
                ray, shape->positions[p], shape->radius[p], uv, distance)) {
          hit      = true;
          element  = shape->bvh->primitives[idx];
          ray.tmax = distance;
        }
      }
    } else if (!shape->lines.empty()) {
      for (auto idx = node.start; idx < node.start + node.num; idx++) {
        auto& l = shape->lines[shape->bvh->primitives[idx]];
        if (intersect_line(ray, shape->positions[l.x], shape->positions[l.y],
                shape->radius[l.x], shape->radius[l.y], uv, distance)) {
          hit      = true;
          element  = shape->bvh->primitives[idx];
          ray.tmax = distance;
        }
      }
    } else if (!shape->triangles.empty()) {
      for (auto idx = node.start; idx < node.start + node.num; idx++) {
        auto& t = shape->triangles[shape->bvh->primitives[idx]];
        if (intersect_triangle(ray, shape->positions[t.x],
                shape->positions[t.y], shape->positions[t.z], uv, distance)) {
          hit      = true;
          element  = shape->bvh->primitives[idx];
          ray.tmax = distance;
        }
      }
    } else if (!shape->quads.empty()) {
      for (auto idx = node.start; idx < node.start + node.num; idx++) {
        auto& q = shape->quads[shape->bvh->primitives[idx]];
        if (intersectPatch(ray, shape->positions[q.x], shape->positions[q.y],
                shape->positions[q.z], shape->positions[q.w], uv, distance)) {
          // if (intersect_quad(ray, shape->positions[q.x],
          // shape->positions[q.y],
          //        shape->positions[q.z], shape->positions[q.w], uv,
          //        distance)){
          hit      = true;
          element  = shape->bvh->primitives[idx];
          ray.tmax = distance;
        }
      }
    }

    // check for early exit
    if (find_any && hit) return hit;
  }

  return hit;
}

// Intersect ray with a bvh->
static bool intersect_scene_bvh(const pathtrace_scene* scene, const ray3f& ray_,
    int& instance, int& element, vec2f& uv, float& distance, bool find_any,
    bool non_rigid_frames) {
  // get bvh and scene pointers for fast access
  auto bvh = scene->bvh;

  // check empty
  if (bvh->nodes.empty()) return false;

  // node stack
  int  node_stack[128];
  auto node_cur          = 0;
  node_stack[node_cur++] = 0;

  // shared variables
  auto hit = false;

  // copy ray to modify it
  auto ray = ray_;

  // prepare ray for fast queries
  auto ray_dinv  = vec3f{1 / ray.d.x, 1 / ray.d.y, 1 / ray.d.z};
  auto ray_dsign = vec3i{(ray_dinv.x < 0) ? 1 : 0, (ray_dinv.y < 0) ? 1 : 0,
      (ray_dinv.z < 0) ? 1 : 0};

  // walking stack
  while (node_cur) {
    // grab node
    auto& node = bvh->nodes[node_stack[--node_cur]];

    // intersect bbox
    // if (!intersect_bbox(ray, ray_dinv, ray_dsign, node.bbox)) continue;
    if (!intersect_bbox(ray, ray_dinv, node.bbox)) continue;

    // intersect node, switching based on node type
    // for each type, iterate over the the primitive list
    if (node.internal) {
      // for internal nodes, attempts to proceed along the
      // split axis from smallest to largest nodes
      if (ray_dsign[node.axis]) {
        node_stack[node_cur++] = node.start + 0;
        node_stack[node_cur++] = node.start + 1;
      } else {
        node_stack[node_cur++] = node.start + 1;
        node_stack[node_cur++] = node.start + 0;
      }
    } else {
      for (auto idx = node.start; idx < node.start + node.num; idx++) {
        auto instance_ = scene->instances[scene->bvh->primitives[idx]];
        auto inv_ray   = transform_ray(
            inverse(instance_->frame, non_rigid_frames), ray);
        if (intersect_shape_bvh(
                instance_->shape, inv_ray, element, uv, distance, find_any)) {
          hit      = true;
          instance = scene->bvh->primitives[idx];
          ray.tmax = distance;
        }
      }
    }

    // check for early exit
    if (find_any && hit) return hit;
  }

  return hit;
}

// Intersect ray with a bvh->
static bool intersect_instance_bvh(const pathtrace_instance* instance,
    const ray3f& ray, int& element, vec2f& uv, float& distance, bool find_any,
    bool non_rigid_frames) {
  auto inv_ray = transform_ray(inverse(instance->frame, non_rigid_frames), ray);
  return intersect_shape_bvh(
      instance->shape, inv_ray, element, uv, distance, find_any);
}

pathtrace_intersection intersect_scene_bvh(const pathtrace_scene* scene,
    const ray3f& ray, bool find_any, bool non_rigid_frames) {
  auto intersection = pathtrace_intersection{};
  intersection.hit  = intersect_scene_bvh(scene, ray, intersection.instance,
      intersection.element, intersection.uv, intersection.distance, find_any,
      non_rigid_frames);
  return intersection;
}
pathtrace_intersection intersect_instance_bvh(
    const pathtrace_instance* instance, const ray3f& ray, bool find_any,
    bool non_rigid_frames) {
  auto intersection = pathtrace_intersection{};
  intersection.hit = intersect_instance_bvh(instance, ray, intersection.element,
      intersection.uv, intersection.distance, find_any, non_rigid_frames);
  return intersection;
}

}  // namespace yocto

// -----------------------------------------------------------------------------
// IMPLEMENTATION FOR PATH TRACING
// -----------------------------------------------------------------------------
namespace yocto {

// Evaluate emission
static vec3f eval_emission(
    const vec3f& emission, const vec3f& normal, const vec3f& outgoing) {
  return emission;
}

// Evaluates/sample the BRDF scaled by the cosine of the incoming direction.
static vec3f eval_brdfcos(const pathtrace_brdf& brdf, const vec3f& normal,
    const vec3f& outgoing, const vec3f& incoming) {
  if (brdf.roughness == 0) return zero3f;

  // accumulate the lobes
  auto brdfcos = zero3f;
  if (brdf.diffuse != zero3f) {
    brdfcos += brdf.diffuse *
               eval_diffuse_reflection(normal, outgoing, incoming);
  }
  if (brdf.specular != zero3f) {
    brdfcos += brdf.specular * eval_microfacet_reflection(brdf.ior,
                                   brdf.roughness, normal, outgoing, incoming);
  }
  if (brdf.metal != zero3f) {
    brdfcos += brdf.metal * eval_microfacet_reflection(brdf.meta, brdf.metak,
                                brdf.roughness, normal, outgoing, incoming);
  }
  if (brdf.transmission != zero3f) {
    brdfcos += brdf.transmission * eval_microfacet_transmission(brdf.ior,
                                       brdf.roughness, normal, outgoing,
                                       incoming);
  }
  if (brdf.refraction != zero3f) {
    brdfcos += brdf.refraction * eval_microfacet_refraction(brdf.ior,
                                     brdf.roughness, normal, outgoing,
                                     incoming);
  }

  return brdfcos;
}

static vec3f eval_delta(const pathtrace_brdf& brdf, const vec3f& normal,
    const vec3f& outgoing, const vec3f& incoming) {
  if (brdf.roughness != 0) return zero3f;

  auto brdfcos = zero3f;

  if (brdf.specular != zero3f && brdf.refraction == zero3f) {
    brdfcos += brdf.specular *
               eval_delta_reflection(brdf.ior, normal, outgoing, incoming);
  }
  if (brdf.metal != zero3f) {
    brdfcos += brdf.metal * eval_delta_reflection(brdf.meta, brdf.metak, normal,
                                outgoing, incoming);
  }
  if (brdf.transmission != zero3f) {
    brdfcos += brdf.transmission *
               eval_delta_transmission(brdf.ior, normal, outgoing, incoming);
  }
  if (brdf.refraction != zero3f) {
    brdfcos += brdf.refraction *
               eval_delta_refraction(brdf.ior, normal, outgoing, incoming);
  }

  return brdfcos;
}

// Picks a direction based on the BRDF
static vec3f sample_brdfcos(const pathtrace_brdf& brdf, const vec3f& normal,
    const vec3f& outgoing, float rnl, const vec2f& rn) {
  if (brdf.roughness == 0) return zero3f;

  auto cdf = 0.0f;

  if (brdf.diffuse_pdf != 0) {
    cdf += brdf.diffuse_pdf;
    if (rnl < cdf) return sample_diffuse_reflection(normal, outgoing, rn);
  }

  if (brdf.specular_pdf != 0 && brdf.refraction_pdf == 0) {
    cdf += brdf.specular_pdf;
    if (rnl < cdf)
      return sample_microfacet_reflection(
          brdf.ior, brdf.roughness, normal, outgoing, rn);
  }

  if (brdf.metal_pdf != 0) {
    cdf += brdf.metal_pdf;
    if (rnl < cdf)
      return sample_microfacet_reflection(
          brdf.meta, brdf.metak, brdf.roughness, normal, outgoing, rn);
  }

  if (brdf.transmission_pdf != 0) {
    cdf += brdf.transmission_pdf;
    if (rnl < cdf)
      return sample_microfacet_transmission(
          brdf.ior, brdf.roughness, normal, outgoing, rn);
  }

  if (brdf.refraction_pdf != 0) {
    cdf += brdf.refraction_pdf;
    if (rnl < cdf)
      return sample_microfacet_refraction(
          brdf.ior, brdf.roughness, normal, outgoing, rnl, rn);
  }

  return zero3f;
}

static vec3f sample_delta(const pathtrace_brdf& brdf, const vec3f& normal,
    const vec3f& outgoing, float rnl) {
  if (brdf.roughness != 0) return zero3f;

  // keep a weight sum to pick a lobe
  auto cdf = 0.0f;
  cdf += brdf.diffuse_pdf;

  if (brdf.specular_pdf != 0 && brdf.refraction_pdf == 0) {
    cdf += brdf.specular_pdf;
    if (rnl < cdf) {
      return sample_delta_reflection(brdf.ior, normal, outgoing);
    }
  }

  if (brdf.metal_pdf != 0) {
    cdf += brdf.metal_pdf;
    if (rnl < cdf) {
      return sample_delta_reflection(brdf.meta, brdf.metak, normal, outgoing);
    }
  }

  if (brdf.transmission_pdf != 0) {
    cdf += brdf.transmission_pdf;
    if (rnl < cdf) {
      return sample_delta_transmission(brdf.ior, normal, outgoing);
    }
  }

  if (brdf.refraction_pdf != 0) {
    cdf += brdf.refraction_pdf;
    if (rnl < cdf) {
      return sample_delta_refraction(brdf.ior, normal, outgoing, rnl);
    }
  }

  return zero3f;
}

// Compute the weight for sampling the BRDF
static float sample_brdfcos_pdf(const pathtrace_brdf& brdf, const vec3f& normal,
    const vec3f& outgoing, const vec3f& incoming) {
  if (brdf.roughness == 0) return 0;

  auto pdf = 0.0f;

  if (brdf.diffuse_pdf != 0) {
    pdf += brdf.diffuse_pdf *
           sample_diffuse_reflection_pdf(normal, outgoing, incoming);
  }

  if (brdf.specular_pdf != 0 && brdf.refraction_pdf == 0) {
    pdf += brdf.specular_pdf * sample_microfacet_reflection_pdf(brdf.ior,
                                   brdf.roughness, normal, outgoing, incoming);
  }

  if (brdf.metal_pdf != 0) {
    pdf += brdf.metal_pdf * sample_microfacet_reflection_pdf(brdf.meta,
                                brdf.metak, brdf.roughness, normal, outgoing,
                                incoming);
  }

  if (brdf.transmission_pdf != 0) {
    pdf += brdf.transmission_pdf * sample_microfacet_transmission_pdf(brdf.ior,
                                       brdf.roughness, normal, outgoing,
                                       incoming);
  }

  if (brdf.refraction_pdf != 0) {
    pdf += brdf.refraction_pdf * sample_microfacet_refraction_pdf(brdf.ior,
                                     brdf.roughness, normal, outgoing,
                                     incoming);
  }

  return pdf;
}

static float sample_delta_pdf(const pathtrace_brdf& brdf, const vec3f& normal,
    const vec3f& outgoing, const vec3f& incoming) {
  if (brdf.roughness != 0) return 0;

  auto pdf = 0.0f;
  if (brdf.specular_pdf != 0 && brdf.refraction_pdf == 0) {
    pdf += brdf.specular_pdf *
           sample_delta_reflection_pdf(brdf.ior, normal, outgoing, incoming);
  }
  if (brdf.metal_pdf != 0) {
    pdf += brdf.metal_pdf * sample_delta_reflection_pdf(brdf.meta, brdf.metak,
                                normal, outgoing, incoming);
  }
  if (brdf.transmission_pdf != 0) {
    pdf += brdf.transmission_pdf *
           sample_delta_transmission_pdf(brdf.ior, normal, outgoing, incoming);
  }
  if (brdf.refraction_pdf != 0) {
    pdf += brdf.refraction_pdf *
           sample_delta_refraction_pdf(brdf.ior, normal, outgoing, incoming);
  }
  return pdf;
}

// Sample lights wrt solid angle
static vec3f sample_lights(const pathtrace_scene* scene, const vec3f& position,
    float rl, float rel, const vec2f& ruv) {
  auto light_id = sample_uniform((int)scene->lights.size(), rl);
  auto light    = scene->lights[light_id];
  if (light->instance != nullptr) {
    auto element = sample_discrete_cdf(light->cdf, rel);
    auto uv      = zero2f;
    if (!light->instance->shape->triangles.empty())
      uv = sample_triangle(ruv);
    else
      uv = ruv;
    auto lposition = eval_position(light->instance, element, uv);
    return normalize(lposition - position);
  } else if (light->environment != nullptr) {
    if (light->environment->emission_tex != nullptr) {
      auto emission_tex = light->environment->emission_tex;
      auto idx          = sample_discrete_cdf(light->cdf, rel);
      auto size         = texture_size(emission_tex);
      auto uv           = vec2f{
          (idx % size.x + 0.5f) / size.x, (idx / size.x + 0.5f) / size.y};
      return transform_direction(light->environment->frame,
          {cos(uv.x * 2 * pif) * sin(uv.y * pif), cos(uv.y * pif),
              sin(uv.x * 2 * pif) * sin(uv.y * pif)});
    } else {
      return sample_sphere(ruv);
    }
  } else {
    return zero3f;
  }
}

// Sample lights pdf
static float sample_lights_pdf(const pathtrace_scene* scene,
    const vec3f& position, const vec3f& direction) {
  auto pdf = 0.0f;
  for (auto& light : scene->lights) {
    if (light->instance) {
      // check all intersection
      auto lpdf          = 0.0f;
      auto next_position = position;
      for (auto bounce = 0; bounce < 100; bounce++) {
        auto intersection = intersect_instance_bvh(
            light->instance, {next_position, direction});
        if (!intersection.hit) break;
        // accumulate pdf
        auto lposition = eval_position(
            light->instance, intersection.element, intersection.uv);
        auto lnormal = eval_element_normal(
            light->instance, intersection.element);
        // prob triangle * area triangle = area triangle mesh
        auto area = light->cdf.back();
        lpdf += distance_squared(lposition, position) /
                (abs(dot(lnormal, direction)) * area);
        // continue
        next_position = lposition + direction * 1e-3f;
      }
      pdf += lpdf;
    } else if (light->environment) {
      if (light->environment->emission_tex) {
        auto emission_tex = light->environment->emission_tex;
        auto size         = texture_size(emission_tex);
        auto wl           = transform_direction(
            inverse(light->environment->frame), direction);
        auto texcoord = vec2f{atan2(wl.z, wl.x) / (2 * pif),
            acos(clamp(wl.y, -1.0f, 1.0f)) / pif};
        if (texcoord.x < 0) texcoord.x += 1;
        auto i    = clamp((int)(texcoord.x * size.x), 0, size.x - 1);
        auto j    = clamp((int)(texcoord.y * size.y), 0, size.y - 1);
        auto prob = sample_discrete_cdf_pdf(light->cdf, j * size.x + i) /
                    light->cdf.back();
        auto angle = (2 * pif / size.x) * (pif / size.y) *
                     sin(pif * (j + 0.5f) / size.y);
        pdf += prob / angle;
      } else {
        pdf += 1 / (4 * pif);
      }
    }
  }
  pdf *= sample_uniform_pdf((int)scene->lights.size());
  return pdf;
}

// YOUR CODE GOES HERE ----------------------------------------------------
static vec3f eval_scattering(
    const pathtrace_vsdf& vsdf, const vec3f& outgoing, const vec3f& incoming) {
  if (vsdf.density == zero3f) return zero3f;
  return vsdf.scatter * vsdf.density *
         eval_phasefunction(vsdf.anisotropy, outgoing, incoming);
}

// YOUR CODE GOES HERE ----------------------------------------------------
static vec3f sample_scattering(const pathtrace_vsdf& vsdf,
    const vec3f& outgoing, float rnl, const vec2f& rn) {
  if (vsdf.density == zero3f) return zero3f;
  return sample_phasefunction(vsdf.anisotropy, outgoing, rn);
}

// YOUR CODE GOES HERE ----------------------------------------------------
static float sample_scattering_pdf(
    const pathtrace_vsdf& vsdf, const vec3f& outgoing, const vec3f& incoming) {
  if (vsdf.density == zero3f) return 0;
  return sample_phasefunction_pdf(vsdf.anisotropy, incoming, outgoing);
}

static vec4f shade_path(const pathtrace_scene* scene, const ray3f& ray_,
    rng_state& rng, const pathtrace_params& params);

// Path tracing.
// YOUR CODE GOES HERE ----------------------------------------------------
static vec4f shade_volpath(const pathtrace_scene* scene, const ray3f& ray_,
    rng_state& rng, const pathtrace_params& params) {
  //<init>
  auto l      = zero3f;
  auto w      = vec3f{1, 1, 1};
  auto ray    = ray_;
  auto hit    = false;
  auto vstack = vector<pathtrace_vsdf>{};

  for (auto bounce = 0; bounce < params.bounces; bounce++) {
    //<intersection and environment>
    auto isec = intersect_scene_bvh(scene, ray);
    if (!isec.hit) {
      l += w * eval_environment(scene, ray);
      break;
    }
    // isec.distance => 1° phase, sample a distance in the medium and compute the distance pdf

    //<sample transmittance>
    auto in_volume = false;

    if (!vstack.empty()) {
        // 2° phase, compute the transmission at a given distance
      auto density  = vstack.back().density;
      auto distance = sample_transmittance(
          density, isec.distance, rand1f(rng), rand1f(rng));
      w *= eval_transmittance(density, distance) /
           sample_transmittance_pdf(density, distance, isec.distance);
      in_volume     = distance < isec.distance;
      isec.distance = distance;
    }

    if (!in_volume) {  //<handle surface>
      auto o      = -ray.d;
      auto object = scene->instances[isec.instance];
      auto p      = eval_position(object, isec.element, isec.uv);
      auto n      = eval_shading_normal(object, isec.element, isec.uv, o);
      auto e      = eval_emission(object, isec.element, isec.uv, n, o);
      auto f      = eval_brdf(object, isec.element, isec.uv, n, o);
      auto op     = f.opacity;

      // <handle opacity>
      if (op < 1 && rand1f(rng) >= op) {
        ray = {p + ray.d * 1e-2f, ray.d};
        bounce -= 1;
        continue;
      }
      hit = true;

      l += w * eval_emission(e, n, o);

      auto i = zero3f;
      if (!is_delta(f)) {
        i = rand1f(rng) < 0.5f
                ? sample_brdfcos(f, n, o, rand1f(rng), rand2f(rng))
                : sample_lights(
                      scene, p, rand1f(rng), rand1f(rng), rand2f(rng));
        w *= eval_brdfcos(f, n, o, i) * 2 /
             (sample_brdfcos_pdf(f, n, o, i) + sample_lights_pdf(scene, p, i));
      } else {
        i = sample_delta(f, n, o, rand1f(rng));
        w *= eval_delta(f, n, o, i) / sample_delta_pdf(f, n, o, i);
      }

      ray = {p, i};

      if (has_volume(object) && dot(n, o) * dot(n, i) < 0) {
        if (vstack.empty())
          vstack.push_back(eval_vsdf(object, isec.element, isec.uv));
        else
          vstack.pop_back();
      }
    } else {  //<handle volume>
      auto  o      = -ray.d;
      auto  object = scene->instances[isec.instance];
      auto  p      = ray.o + ray.d * isec.distance;
      auto& vol    = vstack.back();

      hit = true;

      // 3° phase, sample the phase function and compute the pdf
      auto i = rand1f(rng) < 0.5f
                   ? sample_scattering(vol, o, rand1f(rng), rand2f(rng))
                   : sample_lights(
                         scene, p, rand1f(rng), rand1f(rng), rand2f(rng));
      w *= eval_scattering(vol, o, i) * 2 /
           (sample_scattering_pdf(vol, o, i) + sample_lights_pdf(scene, p, i));
      ray = {p, i};
    }

    // check weight
    if (w == zero3f || !isfinite(w)) break;

    //<russian roulette>
    if (bounce > 3) {
      auto rr_p = min((float)0.99, max(w));
      if (rand1f(rng) >= rr_p) break;
      w *= 1 / rr_p;
    }
    //<recurse>
  }
  return {l.x, l.y, l.z, hit ? 1.0f : 0.0f};
}

// Path tracing.
static vec4f shade_path(const pathtrace_scene* scene, const ray3f& ray_,
    rng_state& rng, const pathtrace_params& params) {
  // initialize
  auto radiance = zero3f;
  auto weight   = vec3f{1, 1, 1};
  auto ray      = ray_;
  auto hit      = false;

  // trace  path
  for (auto bounce = 0; bounce < params.bounces; bounce++) {
    // intersect next point
    auto intersection = intersect_scene_bvh(scene, ray);
    if (!intersection.hit) {
      radiance += weight * eval_environment(scene, ray);
      break;
    }

    // prepare shading point
    auto outgoing = -ray.d;
    auto instance = scene->instances[intersection.instance];
    auto element  = intersection.element;
    auto uv       = intersection.uv;
    auto position = eval_position(instance, element, uv);
    auto normal   = eval_shading_normal(instance, element, uv, outgoing);
    auto emission = eval_emission(instance, element, uv, normal, outgoing);
    auto brdf     = eval_brdf(instance, element, uv, normal, outgoing);

    // handle opacity
    if (brdf.opacity < 1 && rand1f(rng) >= brdf.opacity) {
      ray = {position + ray.d * 1e-2f, ray.d};
      bounce -= 1;
      continue;
    }
    hit = true;

    // accumulate emission
    radiance += weight * eval_emission(emission, normal, outgoing);

    // next direction
    auto incoming = zero3f;
    if (!is_delta(brdf)) {
      if (rand1f(rng) < 0.5f) {
        incoming = sample_brdfcos(
            brdf, normal, outgoing, rand1f(rng), rand2f(rng));
      } else {
        incoming = sample_lights(
            scene, position, rand1f(rng), rand1f(rng), rand2f(rng));
      }
      weight *= eval_brdfcos(brdf, normal, outgoing, incoming) /
                (0.5f * sample_brdfcos_pdf(brdf, normal, outgoing, incoming) +
                    0.5f * sample_lights_pdf(scene, position, incoming));
    } else {
      incoming = sample_delta(brdf, normal, outgoing, rand1f(rng));
      weight *= eval_delta(brdf, normal, outgoing, incoming) /
                sample_delta_pdf(brdf, normal, outgoing, incoming);
    }

    // setup next iteration
    ray = {position, incoming};

    // check weight
    if (weight == zero3f || !isfinite(weight)) break;

    // russian roulette
    if (bounce > 3) {
      auto rr_prob = min((float)0.99, max(weight));
      if (rand1f(rng) >= rr_prob) break;
      weight *= 1 / rr_prob;
    }
  }

  return {radiance.x, radiance.y, radiance.z, hit ? 1.0f : 0.0f};
}

// Recursive path tracing.
static vec4f shade_naive(const pathtrace_scene* scene, const ray3f& ray_,
    rng_state& rng, const pathtrace_params& params) {
  // initialize
  auto radiance = zero3f;
  auto weight   = vec3f{1, 1, 1};
  auto ray      = ray_;
  auto hit      = false;

  // trace  path
  for (auto bounce = 0; bounce < params.bounces; bounce++) {
    // intersect next point
    auto intersection = intersect_scene_bvh(scene, ray);
    if (!intersection.hit) {
      radiance += weight * eval_environment(scene, ray);
      break;
    }

    // prepare shading point
    auto outgoing = -ray.d;
    auto instance = scene->instances[intersection.instance];
    auto element  = intersection.element;
    auto uv       = intersection.uv;
    auto position = eval_position(instance, element, uv);
    auto normal   = eval_shading_normal(instance, element, uv, outgoing);
    auto emission = eval_emission(instance, element, uv, normal, outgoing);
    auto brdf     = eval_brdf(instance, element, uv, normal, outgoing);

    // handle opacity
    if (brdf.opacity < 1 && rand1f(rng) >= brdf.opacity) {
      ray = {position + ray.d * 1e-2f, ray.d};
      bounce -= 1;
      continue;
    }
    hit = true;

    // accumulate emission
    radiance += weight * eval_emission(emission, normal, outgoing);

    // next direction
    auto incoming = zero3f;
    if (!is_delta(brdf)) {
      incoming = sample_brdfcos(
          brdf, normal, outgoing, rand1f(rng), rand2f(rng));
      weight *= eval_brdfcos(brdf, normal, outgoing, incoming) /
                sample_brdfcos_pdf(brdf, normal, outgoing, incoming);
    } else {
      incoming = sample_delta(brdf, normal, outgoing, rand1f(rng));
      weight *= eval_delta(brdf, normal, outgoing, incoming) /
                sample_delta_pdf(brdf, normal, outgoing, incoming);
    }

    // setup next iteration
    ray = {position, incoming};

    // check weight
    if (weight == zero3f || !isfinite(weight)) break;

    // russian roulette
    if (bounce > 3) {
      auto rr_prob = min((float)0.99, max(weight));
      if (rand1f(rng) >= rr_prob) break;
      weight *= 1 / rr_prob;
    }
  }

  return {radiance.x, radiance.y, radiance.z, hit ? 1.0f : 0.0f};
}

// Eyelight for quick previewing.
static vec4f shade_eyelight(const pathtrace_scene* scene, const ray3f& ray_,
    rng_state& rng, const pathtrace_params& params) {
  // initialize
  auto radiance = zero3f;
  auto weight   = vec3f{1, 1, 1};
  auto ray      = ray_;
  auto hit      = false;

  // trace  path
  for (auto bounce = 0; bounce < max(params.bounces, 4); bounce++) {
    // intersect next point
    auto intersection = intersect_scene_bvh(scene, ray);
    if (!intersection.hit) {
      radiance += weight * eval_environment(scene, ray);
      break;
    }

    // prepare shading point
    auto outgoing = -ray.d;
    auto instance = scene->instances[intersection.instance];
    auto element  = intersection.element;
    auto uv       = intersection.uv;
    auto position = eval_position(instance, element, uv);
    auto normal   = eval_shading_normal(instance, element, uv, outgoing);
    auto emission = eval_emission(instance, element, uv, normal, outgoing);
    auto brdf     = eval_brdf(instance, element, uv, normal, outgoing);

    // handle opacity
    if (brdf.opacity < 1 && rand1f(rng) >= brdf.opacity) {
      ray = {position + ray.d * 1e-2f, ray.d};
      bounce -= 1;
      continue;
    }
    hit = true;

    // accumulate emission
    radiance += weight * eval_emission(emission, normal, outgoing);

    // brdf * light
    auto incoming = outgoing;
    radiance += weight * pif * eval_brdfcos(brdf, normal, outgoing, incoming);

    // continue path
    if (!is_delta(brdf)) break;
    incoming = sample_delta(brdf, normal, outgoing, rand1f(rng));
    weight *= eval_delta(brdf, normal, outgoing, incoming) /
              sample_delta_pdf(brdf, normal, outgoing, incoming);
    if (weight == zero3f || !isfinite(weight)) break;

    // setup next iteration
    ray = {position, incoming};
  }

  return {radiance.x, radiance.y, radiance.z, hit ? 1.0f : 0.0f};
}

// Trace a single ray from the camera using the given algorithm.
using pathtrace_shader_func = vec4f (*)(const pathtrace_scene* scene,
    const ray3f& ray, rng_state& rng, const pathtrace_params& params);
static pathtrace_shader_func get_shader(const pathtrace_params& params) {
  switch (params.shader) {
    case pathtrace_shader_type::naive: return shade_naive;
    case pathtrace_shader_type::path: return shade_path;
    case pathtrace_shader_type::eyelight: return shade_eyelight;
    case pathtrace_shader_type::volpath: return shade_volpath;
    default: {
      throw std::runtime_error("sampler unknown");
      return nullptr;
    }
  }
}

// Trace a block of samples
void render_sample(pathtrace_state* state, const pathtrace_scene* scene,
    const pathtrace_camera* camera, const vec2i& ij,
    const pathtrace_params& params) {
  auto shader = get_shader(params);
  auto ray    = sample_camera(camera, ij, state->render.imsize(),
      rand2f(state->rngs[ij]), rand2f(state->rngs[ij]));
  auto shaded = shader(scene, ray, state->rngs[ij], params);
  if (!isfinite(xyz(shaded))) shaded = {shaded.x, shaded.y, shaded.z, 1};
  if (max(xyz(shaded)) > params.clamp) {
    auto scale = params.clamp / max(xyz(shaded));
    shaded = {shaded.x * scale, shaded.y * scale, shaded.z * scale, shaded.w};
  }
  state->accumulation[ij] += shaded;
  state->samples[ij] += 1;
  state->render[ij] = state->accumulation[ij] / state->samples[ij];
}

// Forward declaration
pathtrace_light* add_light(pathtrace_scene* scene);

// Init trace lights
void init_lights(pathtrace_scene* scene, const pathtrace_params& params,
    progress_callback progress_cb) {
  // handle progress
  auto progress = vec2i{0, 1};
  if (progress_cb) progress_cb("build light", progress.x++, progress.y);

  for (auto light : scene->lights) delete light;
  scene->lights.clear();

  for (auto instance : scene->instances) {
    if (instance->material->emission == zero3f) continue;
    auto shape = instance->shape;
    if (shape->triangles.empty() && shape->quads.empty()) continue;
    if (progress_cb) progress_cb("build light", progress.x++, ++progress.y);
    auto light      = add_light(scene);
    light->instance = instance;
    // light->environment = nullptr;
    if (!shape->triangles.empty()) {
      light->cdf = vector<float>(shape->triangles.size());
      for (auto idx = 0; idx < light->cdf.size(); idx++) {
        auto& t         = shape->triangles[idx];
        light->cdf[idx] = triangle_area(shape->positions[t.x],
            shape->positions[t.y], shape->positions[t.z]);
        if (idx != 0) light->cdf[idx] += light->cdf[idx - 1];
      }
    }
    if (!shape->quads.empty()) {
      light->cdf = vector<float>(shape->quads.size());
      for (auto idx = 0; idx < light->cdf.size(); idx++) {
        auto& t         = shape->quads[idx];
        light->cdf[idx] = quad_area(shape->positions[t.x],
            shape->positions[t.y], shape->positions[t.z],
            shape->positions[t.w]);
        if (idx != 0) light->cdf[idx] += light->cdf[idx - 1];
      }
    }
  }
  for (auto environment : scene->environments) {
    if (environment->emission == zero3f) continue;
    if (progress_cb) progress_cb("build light", progress.x++, ++progress.y);
    auto light         = add_light(scene);
    light->environment = environment;
    if (environment->emission_tex) {
      auto texture = environment->emission_tex;
      auto size    = texture_size(texture);
      light->cdf   = vector<float>(size.x * size.y);
      for (auto i = 0; i < light->cdf.size(); i++) {
        auto ij       = vec2i{i % size.x, i / size.x};
        auto th       = (ij.y + 0.5f) * pif / size.y;
        auto value    = lookup_texture(texture, ij);
        light->cdf[i] = max(value) * sin(th);
        if (i) light->cdf[i] += light->cdf[i - 1];
      }
    }
  }

  // handle progress
  if (progress_cb) progress_cb("build light", progress.x++, progress.y);
}

// Init a sequence of random number generators.
void init_state(pathtrace_state* state, const pathtrace_scene* scene,
    const pathtrace_camera* camera, const pathtrace_params& params) {
  auto image_size =
      (camera->film.x > camera->film.y)
          ? vec2i{params.resolution,
                (int)round(params.resolution * camera->film.y / camera->film.x)}
          : vec2i{
                (int)round(params.resolution * camera->film.x / camera->film.y),
                params.resolution};
  if (!state->stop) state->pixels.assign(image_size, pixel{});
  if (!state->stop) state->render.assign(image_size, zero4f);
  if (!state->stop) state->odd_render.assign(image_size, zero4f);
  state->accumulation.assign(image_size, zero4f);
  state->samples.assign(image_size, 0);
  state->rngs.assign(image_size, {});
  auto init_rng = make_rng(1301081);
  for (auto& rng : state->rngs) {
    rng = make_rng(params.seed, rand1i(init_rng, 1 << 31) / 2 + 1);
  }
}

// Progressively compute an image by calling render_samples multiple times.
void render_samples(pathtrace_state* state, const pathtrace_scene* scene,
    const pathtrace_camera* camera, const pathtrace_params& params) {
  if (params.noparallel) {
    for (auto j = 0; j < state->render.imsize().y; j++) {
      for (auto i = 0; i < state->render.imsize().x; i++) {
        render_sample(state, scene, camera, {i, j}, params);
      }
    }
  } else {
    parallel_for(state->render.imsize().x, state->render.imsize().y,
        [state, scene, camera, &params](int i, int j) {
          render_sample(state, scene, camera, {i, j}, params);
        });
  }
}

}  // namespace yocto

// -----------------------------------------------------------------------------
// IMPLEMENTATION FOR SCENE TESSELATION
// -----------------------------------------------------------------------------
namespace yocto {

// perform one level of subdivision and modify
// YOUR CODE GOES HERE ----------------------------------------------------
template <typename T>
static void tesselate_catmullclark(
    std::vector<vec4i>& quads, std::vector<T>& vert, bool lock_boundary) {
    if (quads.empty() || vert.empty()) return;
  //<initialize edges>
  // construct edge map and get edges and boundary
  auto emap     = make_edge_map(quads);
  auto edges    = get_edges(emap);
  auto boundary = get_boundary(emap);
  // initialize number of elements
  auto nv = (int)vert.size();
  auto ne = (int)edges.size();
  auto nb = (int)boundary.size();
  auto nf = (int)quads.size();

  //<create vertices>    phase 1
  auto tverts = vector<T>();
  for (auto v : vert) tverts.push_back(v);
  for (auto e : edges) tverts.push_back((vert[e.x] + vert[e.y]) / 2);
  for (auto q : quads)
    if (q.z != q.w)
      tverts.push_back((vert[q.x] + vert[q.y] + vert[q.z] + vert[q.w]) / 4);
    else
      tverts.push_back((vert[q.x] + vert[q.y] + vert[q.z]) / 3);

  //<create faces>
  auto tquads = vector<vec4i>();
  for (auto i = 0; i < quads.size(); i++) {
    if (quads[i].z != quads[i].w) {
      tquads.push_back(
          {quads[i].x, nv + edge_index(emap, {quads[i].x, quads[i].y}),
              nv + ne + i, nv + edge_index(emap, {quads[i].w, quads[i].x})});
      tquads.push_back(
          {quads[i].y, nv + edge_index(emap, {quads[i].y, quads[i].z}),
              nv + ne + i, nv + edge_index(emap, {quads[i].x, quads[i].y})});
      tquads.push_back(
          {quads[i].z, nv + edge_index(emap, {quads[i].z, quads[i].w}),
              nv + ne + i, nv + edge_index(emap, {quads[i].y, quads[i].z})});
      tquads.push_back(
          {quads[i].w, nv + edge_index(emap, {quads[i].w, quads[i].x}),
              nv + ne + i, nv + edge_index(emap, {quads[i].z, quads[i].w})});
    } else {
      tquads.push_back(
          {quads[i].x, nv + edge_index(emap, {quads[i].x, quads[i].y}),
              nv + ne + i, nv + edge_index(emap, {quads[i].z, quads[i].x})});
      tquads.push_back(
          {quads[i].y, nv + edge_index(emap, {quads[i].y, quads[i].z}),
              nv + ne + i, nv + edge_index(emap, {quads[i].x, quads[i].y})});
      tquads.push_back(
          {quads[i].z, nv + edge_index(emap, {quads[i].z, quads[i].x}),
              nv + ne + i, nv + edge_index(emap, {quads[i].y, quads[i].z})});
    }
  }

  //<setup boundary>
  auto tboundary = vector<vec2i>();
  for (auto e : boundary) {
    tboundary.push_back({e.x, nv + edge_index(emap, e)});
    tboundary.push_back({nv + edge_index(emap, e), e.y});
  }

  auto tcrease_edges = vector<vec2i>();
  auto tcrease_verts = vector<int>();
  if (lock_boundary) {
    for (auto& b : tboundary) {
      tcrease_verts.push_back(b.x);
      tcrease_verts.push_back(b.y);
    }
  } else {
    for (auto& b : tboundary) tcrease_edges.push_back(b);
  }

  auto tverts_val = vector<int>(tverts.size(), 2);
  for (auto& e : tboundary) {
    tverts_val[e.x] = (lock_boundary) ? 0 : 1;
    tverts_val[e.y] = (lock_boundary) ? 0 : 1;
  }

  //<averaging>          phase 2
  auto avert  = vector<T>(tverts.size(), T());
  auto acount = vector<int>(tverts.size(), 0);
  for (auto p : tcrease_verts) {
    if (tverts_val[p] != 0) continue;
    avert[p] += tverts[p];
    acount[p] += 1;
  }

  for (auto& e : tcrease_edges) {
    auto c = (tverts[e.x] + tverts[e.y]) / 2;
    for (auto vid : e) {
      if (tverts_val[vid] != 1) continue;
      avert[vid] += c;
      acount[vid] += 1;
    }
  }

  for (auto& q : tquads) {
    auto c = (tverts[q.x] + tverts[q.y] + tverts[q.z] + tverts[q.w]) / 4;
    for (auto vid : q) {
      if (tverts_val[vid] != 2) continue;
      avert[vid] += c;
      acount[vid] += 1;
    }
  }

  for (auto i = 0; i < tverts.size(); i++) avert[i] /= (float)acount[i];

  //<correction>         phase 3
  for (auto i = 0; i < tverts.size(); i++) {
    if (tverts_val[i] != 2) continue;
    avert[i] = tverts[i] + (avert[i] - tverts[i]) * (4 / (float)acount[i]);
  }

  tverts = avert;

  quads = tquads;
  vert  = tverts;
}

void tesselate_shape(pathtrace_shape* shape) {
  if (shape->subdivisions != 0) {
    if (!shape->quadspos.empty()) {
      for (auto level = 0; level < shape->subdivisions; level++)
        tesselate_catmullclark(shape->quadspos, shape->positions, false);
      for (auto level = 0; level < shape->subdivisions; level++)
        tesselate_catmullclark(shape->quadstexcoord, shape->texcoords, true);
      if (shape->smooth) {
        shape->normals   = compute_normals(shape->quadspos, shape->positions);
        shape->quadsnorm = shape->quadspos;
      } else {
        shape->normals   = {};
        shape->quadsnorm = {};
      }
    } else {
      throw std::runtime_error("not supported yet");
    }
    shape->subdivisions = 0;
  }

  if (!shape->quadspos.empty()) {
    auto [quads, positions, normals, texcoords] = split_facevarying(
        shape->quadspos, shape->quadsnorm, shape->quadstexcoord,
        shape->positions, shape->normals, shape->texcoords);
    shape->positions     = positions;
    shape->normals       = normals;
    shape->texcoords     = texcoords;
    shape->triangles     = quads_to_triangles(quads);
    shape->quadspos      = {};
    shape->quadsnorm     = {};
    shape->quadstexcoord = {};
    shape->points        = {};
    shape->lines         = {};
    shape->radius        = {};
  }

  if (shape->displacement != 0 && shape->displacement_tex != nullptr) {
    if (!shape->triangles.empty() || !shape->quads.empty()) {
      if (shape->normals.empty())
        shape->normals = !shape->triangles.empty()
                             ? compute_normals(
                                   shape->triangles, shape->positions)
                             : compute_normals(shape->quads, shape->positions);
      for (auto idx = 0; idx < shape->positions.size(); idx++) {
        auto disp = mean(xyz(eval_texture(
            shape->displacement_tex, shape->texcoords[idx], true)));
        if (!shape->displacement_tex->ldr.empty()) disp -= 0.5f;
        shape->positions[idx] += shape->normals[idx] * shape->displacement *
                                 disp;
      }
      if (shape->smooth) {
        shape->normals = !shape->triangles.empty()
                             ? compute_normals(
                                   shape->triangles, shape->positions)
                             : compute_normals(shape->quads, shape->positions);
      } else {
        shape->normals = {};
      }
    } else if (!shape->quadspos.empty()) {
      // facevarying case
      auto offset = vector<float>(shape->positions.size(), 0);
      auto count  = vector<int>(shape->positions.size(), 0);
      for (auto fid = 0; fid < shape->quadspos.size(); fid++) {
        auto qpos = shape->quadspos[fid];
        auto qtxt = shape->quadstexcoord[fid];
        for (auto i = 0; i < 4; i++) {
          auto disp = mean(eval_texture(
              shape->displacement_tex, shape->texcoords[qtxt[i]], true));
          if (!shape->displacement_tex->ldr.empty()) disp -= 0.5f;
          offset[qpos[i]] += shape->displacement * disp;
          count[qpos[i]] += 1;
        }
      }
      auto normals = compute_normals(shape->quadspos, shape->positions);
      for (auto vid = 0; vid < shape->positions.size(); vid++) {
        shape->positions[vid] += normals[vid] * offset[vid] / count[vid];
      }
      if (shape->smooth || !shape->normals.empty()) {
        shape->quadsnorm = shape->quadspos;
        shape->normals   = compute_normals(shape->quadspos, shape->positions);
      }
    }

    shape->displacement     = 0;
    shape->displacement_tex = nullptr;
  }
}

void tesselate_shapes(pathtrace_scene* scene, const pathtrace_params& params,
    progress_callback progress_cb) {
  // handle progress
  auto progress = vec2i{0, (int)scene->shapes.size()};

  // tesselate shapes
  for (auto shape : scene->shapes) {
    if (progress_cb) progress_cb("tesselate shape", progress.x++, progress.y);
    tesselate_shape(shape);
  }

  // done
  if (progress_cb) progress_cb("tesselate shape", progress.x++, progress.y);
}

// Initialize subdivision surfaces
void init_subdivs(pathtrace_scene* scene, const pathtrace_params& params,
    progress_callback progress_cb) {
  tesselate_shapes(scene, params, progress_cb);
}

}  // namespace yocto

// -----------------------------------------------------------------------------
// SCENE CREATION
// -----------------------------------------------------------------------------
namespace yocto {

// cleanup
pathtrace_shape::~pathtrace_shape() {
  if (bvh) delete bvh;
}

// cleanup
pathtrace_scene::~pathtrace_scene() {
  if (bvh) delete bvh;
  for (auto camera : cameras) delete camera;
  for (auto instance : instances) delete instance;
  for (auto shape : shapes) delete shape;
  for (auto material : materials) delete material;
  for (auto texture : textures) delete texture;
  for (auto environment : environments) delete environment;
  for (auto light : lights) delete light;
}

// Add element
pathtrace_camera* add_camera(pathtrace_scene* scene) {
  return scene->cameras.emplace_back(new pathtrace_camera{});
}
pathtrace_texture* add_texture(pathtrace_scene* scene) {
  return scene->textures.emplace_back(new pathtrace_texture{});
}
pathtrace_shape* add_shape(pathtrace_scene* scene) {
  return scene->shapes.emplace_back(new pathtrace_shape{});
}
pathtrace_material* add_material(pathtrace_scene* scene) {
  return scene->materials.emplace_back(new pathtrace_material{});
}
pathtrace_instance* add_instance(pathtrace_scene* scene) {
  return scene->instances.emplace_back(new pathtrace_instance{});
}
pathtrace_environment* add_environment(pathtrace_scene* scene) {
  return scene->environments.emplace_back(new pathtrace_environment{});
}
pathtrace_light* add_light(pathtrace_scene* scene) {
  return scene->lights.emplace_back(new pathtrace_light{});
}

// Set cameras
void set_frame(pathtrace_camera* camera, const frame3f& frame) {
  camera->frame = frame;
}
void set_lens(pathtrace_camera* camera, float lens, float aspect, float film) {
  camera->lens = lens;
  camera->film = aspect >= 1 ? vec2f{film, film / aspect}
                             : vec2f{film * aspect, film};
}
void set_focus(pathtrace_camera* camera, float aperture, float focus) {
  camera->aperture = aperture;
  camera->focus    = focus;
}

// Add texture
void set_texture(pathtrace_texture* texture, const image<vec4b>& img) {
  texture->ldr = img;
  texture->hdr = {};
}
void set_texture(pathtrace_texture* texture, const image<vec4f>& img) {
  texture->ldr = {};
  texture->hdr = img;
}

// Add shape
void set_points(pathtrace_shape* shape, const vector<int>& points) {
  shape->points = points;
}
void set_lines(pathtrace_shape* shape, const vector<vec2i>& lines) {
  shape->lines = lines;
}
void set_triangles(pathtrace_shape* shape, const vector<vec3i>& triangles) {
  shape->triangles = triangles;
}

// Ray-Patch Intersection
void set_quads(pathtrace_shape* shape, const vector<vec4i>& quads) {
  shape->quads = quads;
}

void set_fvquads(pathtrace_shape* shape, const vector<vec4i>& quadspos,
    const vector<vec4i>& quadsnorm, const vector<vec4i>& quadstexcoord) {
  shape->quadspos      = quadspos;
  shape->quadsnorm     = quadsnorm;
  shape->quadstexcoord = quadstexcoord;
}
void set_positions(pathtrace_shape* shape, const vector<vec3f>& positions) {
  shape->positions = positions;
}
void set_normals(pathtrace_shape* shape, const vector<vec3f>& normals) {
  shape->normals = normals;
}
void set_texcoords(pathtrace_shape* shape, const vector<vec2f>& texcoords) {
  shape->texcoords = texcoords;
}
void set_radius(pathtrace_shape* shape, const vector<float>& radius) {
  shape->radius = radius;
}
void set_subdivision(
    pathtrace_shape* shape, int subdivisions, bool catmullclark, bool smooth) {
  shape->subdivisions = subdivisions;
  shape->catmullclark = catmullclark;
  shape->smooth       = smooth;
}
void set_displacement(pathtrace_shape* shape, float displacement,
    pathtrace_texture* displacement_tex) {
  shape->displacement     = displacement;
  shape->displacement_tex = displacement_tex;
}

// Add instance
void set_frame(pathtrace_instance* instance, const frame3f& frame) {
  instance->frame = frame;
}
void set_shape(pathtrace_instance* instance, pathtrace_shape* shape) {
  instance->shape = shape;
}
void set_material(pathtrace_instance* instance, pathtrace_material* material) {
  instance->material = material;
}

// Add material
void set_emission(pathtrace_material* material, const vec3f& emission,
    pathtrace_texture* emission_tex) {
  material->emission     = emission;
  material->emission_tex = emission_tex;
}
void set_color(pathtrace_material* material, const vec3f& color,
    pathtrace_texture* color_tex) {
  material->color     = color;
  material->color_tex = color_tex;
}
void set_specular(pathtrace_material* material, float specular,
    pathtrace_texture* specular_tex) {
  material->specular     = specular;
  material->specular_tex = specular_tex;
}
void set_metallic(pathtrace_material* material, float metallic,
    pathtrace_texture* metallic_tex) {
  material->metallic     = metallic;
  material->metallic_tex = metallic_tex;
}
void set_ior(pathtrace_material* material, float ior) { material->ior = ior; }
void set_transmission(pathtrace_material* material, float transmission,
    bool thin, float trdepth, pathtrace_texture* transmission_tex) {
  material->transmission     = transmission;
  material->thin             = thin;
  material->trdepth          = trdepth;
  material->transmission_tex = transmission_tex;
}
void set_thin(pathtrace_material* material, bool thin) {
  material->thin = thin;
}
void set_roughness(pathtrace_material* material, float roughness,
    pathtrace_texture* roughness_tex) {
  material->roughness     = roughness;
  material->roughness_tex = roughness_tex;
}
void set_opacity(pathtrace_material* material, float opacity,
    pathtrace_texture* opacity_tex) {
  material->opacity     = opacity;
  material->opacity_tex = opacity_tex;
}
void set_scattering(pathtrace_material* material, const vec3f& scattering,
    float scanisotropy, pathtrace_texture* scattering_tex) {
  material->scattering     = scattering;
  material->scanisotropy   = scanisotropy;
  material->scattering_tex = scattering_tex;
}
void set_normalmap(
    pathtrace_material* material, pathtrace_texture* normal_tex) {
  material->normal_tex = normal_tex;
}

// Add environment
void set_frame(pathtrace_environment* environment, const frame3f& frame) {
  environment->frame = frame;
}
void set_emission(pathtrace_environment* environment, const vec3f& emission,
    pathtrace_texture* emission_tex) {
  environment->emission     = emission;
  environment->emission_tex = emission_tex;
}

}  // namespace yocto

// -----------------------------------------------------------------------------
// Ray-Patch Intersection
// -----------------------------------------------------------------------------
namespace yocto {

bool intersectPatch(const ray3f& ray, const vec3f& q0, const vec3f& q1,
    const vec3f& q2, const vec3f& q3, vec2f& uv, float& distance) {
  auto state = false;

  auto q00 = q0, q10 = q1, q11 = q2, q01 = q3;
  auto e10 = q10 - q00;
  auto e11 = q11 - q10;
  auto e00 = q01 - q00;
  auto qn  = cross(q10 - q00, q01 - q11);
  q00 -= ray.o;
  q10 -= ray.o;

  auto a = dot(cross(q00, ray.d), e00);
  auto c = dot(qn, ray.d);
  auto b = dot(cross(q10, ray.d), e11);
  b -= a + c;
  auto det = b * b - 4 * a * c;

  if (det < 0) return state;

  det     = sqrt(det);
  auto u1 = .0f, u2 = .0f;
  auto intersect_uv = vec2f{.0f, .0f};
  auto t            = ray.tmax;

  if (c == 0) {
    u1 = -a / b;
    u2 = -1;
  } else {
    u1 = (-b - copysignf(det, b)) / 2;
    u2 = a / u1;
    u1 /= c;
  }

  if (0 <= u1 && u1 <= 1) {
    auto pa = lerp(q00, q10, u1);
    auto pb = lerp(e00, e11, u1);
    auto n  = cross(ray.d, pb);
    det     = dot(n, n);
    n       = cross(n, pa);
    auto t1 = dot(n, pb);
    auto v1 = dot(n, ray.d);
    if (0 <= v1 && v1 <= det && t1 > 0) {
      t            = t1 / det;
      intersect_uv = {u1, v1 / det};
      state        = true;
    }
  }

  if (0 <= u2 && u2 <= 1) {
    auto pa = lerp(q00, q10, u2);
    auto pb = lerp(e00, e11, u2);
    auto n  = cross(ray.d, pb);
    det     = dot(n, n);
    n       = cross(n, pa);
    auto t2 = dot(n, pb) / det;
    auto v2 = dot(n, ray.d);
    if (0 <= v2 && v2 <= det && t > t2 && t2 > 0) {
      t            = t2;
      intersect_uv = {u2, v2 / -det};
      state        = true;
    }
  }

  if (t <= ray.tmax && t > ray.tmin) {
    uv       = intersect_uv;
    distance = t;
    return state;
  }
  return state;
}

}  // namespace yocto

// -----------------------------------------------------------------------------
// Adaptive rendering
// -----------------------------------------------------------------------------

namespace yocto {

void create_sample_spread(
    std::vector<sample_spread>& spread_vec, const float step_q) {
  int radius = 10;
  if (step_q <= 1.99) {
    radius = 4;
  } else if (step_q <= 3.99) {
    radius = 2;
  } else {
    radius = 1;
  }

  spread_vec.clear();
  for (auto i = -radius; i <= radius; i++) {
    for (auto j = -radius; j <= radius; j++) {
      if (i == 0 && j == 0) continue;

      sample_spread spread = {};
      spread.x             = i;
      spread.y             = j;
      spread.div           = 1;
      if (radius == 1) {
        spread_vec.emplace_back(spread);
      } else {
        float dist = sqrtf((float)(i * i) + (j * j));
        if (dist <= radius) spread_vec.emplace_back(spread);
      }
    }
  }
}

std::vector<vec2i> all_image_ij(pathtrace_state* state) {
  std::vector<vec2i> to_return = std::vector<vec2i>{};

  auto size = state->render.imsize();

  for (int j = 0; j < size.y; j++) {
    for (int i = 0; i < size.x; i++) {
      to_return.push_back({i, j});
    }
  }

  return to_return;
}

template <typename Func>
inline void parallel_pixels_in_list(pathtrace_state* state_ptr,
    const pathtrace_params& params, const std::vector<vec2i>& ij_list,
    Func&& func) {
  auto             nthreads = std::thread::hardware_concurrency();
  auto             futures  = std::vector<std::future<void>>(nthreads);
  std::atomic<int> next_idx(0);

  // sample cada pixel selecionado
  futures.clear();
  next_idx.store(0);
  for (auto thread_id = 0; thread_id < nthreads; thread_id++) {
    if (state_ptr->stop) {
      break;
    }

    futures.emplace_back(std::async(
        std::launch::async, [state_ptr, params, &next_idx, &ij_list, &func]() {
          while (!checkEnd(state_ptr, params)) {
            auto idx = next_idx.fetch_add(1);
            if (idx >= ij_list.size()) break;
            func(ij_list[idx]);
          }
        }));
  }

  for (auto& f : futures) f.get();
}

bool checkEnd(pathtrace_state* state, const pathtrace_params& params) {
  if (state->stop) {
    return true;
  }

  if (params.desired_spp > 0) {
    auto img_size  = state->render.imsize().x * state->render.imsize().y;
    auto image_spp = state->sample_count / img_size;
    if (image_spp >= params.desired_spp) {
      return true;
    }
  }

  if (params.desired_seconds > 0) {
    auto elapsed = std::chrono::system_clock::now() - state->start_time;
    auto seconds =
        std::chrono::duration_cast<std::chrono::seconds>(elapsed).count();
    if (seconds >= params.desired_seconds) {
      return true;
    }
  }

  // only check quality if not checking time or spp
  if (params.desired_spp == 0 && params.desired_seconds == 0 &&
      state->min_q >= params.desired_q) {
    return true;
  }

  return false;
}

void trace_until_quality(pathtrace_state* state, const pathtrace_scene* scene,
    const pathtrace_camera* camera, const vec2i& ij,
    const pathtrace_params& params, float q) {
  auto& pixel = state->pixels[ij];

  trace_sample(state, scene, camera, ij, params.sample_step, params);
  if (checkEnd(state, params)) {
    return;
  }

  while (pixel.q < q) {
    trace_sample(state, scene, camera, ij, params.sample_step, params);
    if (checkEnd(state, params)) {
      return;
    }
  }
}

void trace_by_budget(pathtrace_state* state, const pathtrace_scene* scene,
    const pathtrace_camera* camera, const vec2i& ij,
    const pathtrace_params& params) {
  auto& pixel = state->pixels[ij];

  trace_sample(state, scene, camera, ij, pixel.sample_budget, params);
  pixel.sample_budget = 0;
}

void trace_image(pathtrace_state* state_ptr, const pathtrace_scene* scene,
    const pathtrace_camera* camera, const pathtrace_params& params,
    progress_callback_2 progress_cb) {
  std::unique_ptr<pathtrace_state> state_guard;

  if (state_ptr == nullptr) {
    state_guard = std::make_unique<pathtrace_state>();
    state_ptr   = state_guard.get();
  }

  vector<sample_spread> spread_vec;
  float                 step_q = 0.0f;
  state_ptr->curr_q            = -2.0f;

  spread_vec = vector<sample_spread>();
  create_sample_spread(spread_vec, step_q);

  vec2i size = state_ptr->render.imsize();

  if (progress_cb)
    progress_cb(state_ptr, "initial samples",
        get_actual_progress(state_ptr, params), get_max_progress(params));

  state_ptr->curr_q = -1.0f;

  for (auto sampled = 0; sampled < params.min_samples;
       sampled += params.sample_step) {
    parallel_pixels_in_list(state_ptr, params, all_image_ij(state_ptr),
        [state_ptr, scene, camera, &params](const vec2i& ij) {
          trace_sample(
              state_ptr, scene, camera, ij, params.sample_step, params);
        });
  }

  float next_batch = state_ptr->curr_q + params.batch_step;
  while (!checkEnd(state_ptr, params)) {
    // select pixels that are below the actual quality step.
    state_ptr->ij_by_q.clear();
    for (int j = 0; j < size.y; j++) {
      for (int i = 0; i < size.x; i++) {
        auto& pixel         = state_ptr->pixels[{i, j}];
        pixel.sample_budget = 0;
        if (pixel.q < step_q) {
          state_ptr->ij_by_q.push_back({i, j});
        }
      }
    }

    if (progress_cb)
      progress_cb(state_ptr, "samples by quality",
          get_actual_progress(state_ptr, params), get_max_progress(params));
    parallel_pixels_in_list(state_ptr, params, state_ptr->ij_by_q,
        [state_ptr, scene, camera, &params, step_q](const vec2i& ij) {
          trace_until_quality(state_ptr, scene, camera, ij, params, step_q);
        });

    // here is supposed that every pixel in image has quality > step_q
    // trace pixels in neighborhood until pixel budget is reached or if quality
    // drops below step_q to restart de process
    state_ptr->ij_by_proximity.clear();
    // find pixels near pixels sampled by quality using indexes definded in
    // 'sample_spread' vector.
    for (auto& ij_sampled : state_ptr->ij_by_q) {
      auto& pixel = state_ptr->pixels[ij_sampled];

      for (auto& neigh_idx : spread_vec) {
        int   k   = ij_sampled.x + neigh_idx.x;
        int   l   = ij_sampled.y + neigh_idx.y;
        float div = neigh_idx.div;

        if (k >= 0 && l >= 0 && k < size.x && l < size.y) {
          auto& pix_neighbor = state_ptr->pixels[{k, l}];

          if ((pix_neighbor.actual.samples + pix_neighbor.sample_budget) <
              (pixel.actual.samples / div)) {
            pix_neighbor.sample_budget = ((float)pixel.actual.samples / div) -
                                         pix_neighbor.actual.samples;
          }
        }
      }
    }

    // Now, find every pixel with budget > 0
    for (int j = 0; j < size.y; j++) {
      for (int i = 0; i < size.x; i++) {
        auto& pixel = state_ptr->pixels[{i, j}];
        if (pixel.sample_budget > 0) {
          state_ptr->ij_by_proximity.push_back({i, j});
        }
      }
    }

    if (progress_cb)
      progress_cb(state_ptr, "samples by proximity",
          get_actual_progress(state_ptr, params), get_max_progress(params));
    // trace samples for each pixel near pixels sampled by quality
    parallel_pixels_in_list(state_ptr, params, state_ptr->ij_by_proximity,
        [state_ptr, scene, camera, &params](const vec2i& ij) {
          trace_by_budget(state_ptr, scene, camera, ij, params);
        });

    // collect important statistis..
    float tmp_min_q = ::std::numeric_limits<float>::max();
    for (int j = 0; j < size.y; j++) {
      for (int i = 0; i < size.x; i++) {
        auto& pixel = state_ptr->pixels[{i, j}];

        if (tmp_min_q > pixel.q) {
          tmp_min_q = pixel.q;
        }
      }
    }

    state_ptr->min_q = tmp_min_q;
    if (state_ptr->min_q >= step_q) {
      state_ptr->curr_q = step_q;

      if (state_ptr->curr_q >= next_batch) {
        next_batch = state_ptr->curr_q + params.batch_step;
      }
      step_q += params.step_q;
      create_sample_spread(spread_vec, step_q);

      if (params.desired_seconds == 0 && params.desired_spp == 0 &&
          params.desired_q > params.desired_q) {
        step_q = params.desired_q;
      }
    }
  }

  if (!state_ptr->stop && progress_cb) {
    progress_cb(state_ptr, "samples by proximity", get_max_progress(params),
        get_max_progress(params));
  }
}

// Trace a block of samples
void trace_sample(pathtrace_state* state, const pathtrace_scene* scene,
    const pathtrace_camera* camera, const vec2i& ij, const int num_samples,
    const pathtrace_params& params) {
  auto& pixel = state->pixels[ij];

  auto sampler = get_shader(params);

  int samples = num_samples;
  if ((pixel.actual.samples + num_samples) > params.max_samples) {
    samples = params.max_samples - pixel.actual.samples;
  }
  for (auto s = 0; s < samples; s++) {
    if (!state->stop.load()) {
      auto start    = std::chrono::steady_clock::now();
      auto ray      = sample_camera(camera, ij, state->render.imsize(),
          rand2f(state->rngs[ij]), rand2f(state->rngs[ij]));
      auto radiance = sampler(scene, ray, state->rngs[ij], params);
      auto hit      = radiance.w == 1.0f ? true : false;
      pixel.time_in_sample +=
          (std::chrono::steady_clock::now() - start).count();
      state->sample_count++;

      if (!hit) {
        if (scene->environments.empty()) {
          radiance = zero4f;
          hit      = false;
        } else {
          hit = true;
        }
      }
      if (!isfinite(radiance)) radiance = zero4f;
      if (max(radiance) >= params.clamp)
        radiance = radiance * (params.clamp / max(radiance));

      /*pixel.actual.radiance += radiance;
      pixel.actual.hits += hit ? 1 : 0;
      pixel.actual.samples += 1;

      if (pixel.actual.samples % 2 == 1) {
        pixel.odd.radiance += radiance;
        pixel.odd.hits += hit ? 1 : 0;
        pixel.odd.samples += 1;
      }*/
      pixel.actual.radiance = {pixel.actual.radiance.x + radiance.x,
          pixel.actual.radiance.y + radiance.y,
          pixel.actual.radiance.z + radiance.z};
      pixel.actual.hits += hit ? 1 : 0;
      pixel.actual.samples += 1;

      if (pixel.actual.samples % 2 == 1) {
        pixel.odd.radiance = {pixel.odd.radiance.x + radiance.x,
            pixel.odd.radiance.y + radiance.y,
            pixel.odd.radiance.z + radiance.z};
        pixel.odd.hits += hit ? 1 : 0;
        pixel.odd.samples += 1;
      }
    }

    if (checkEnd(state, params)) {
      return;
    }
  }

  /*if (pixel.actual.hits) {
    pixel.actual.radiance.w /= (float)pixel.actual.samples;
    state->render[ij] = pixel.actual.radiance;
  } else {
    state->render[ij] = zero4f;
  }
  if (pixel.odd.hits) {
    pixel.odd.radiance.w /= (float)pixel.odd.samples;
    state->odd_render[ij] = pixel.odd.radiance / (float)pixel.odd.samples;
  } else {
    state->render[ij] = zero4f;
  }*/
  if (pixel.actual.hits) {
    state->render[ij] = {float(pixel.actual.radiance.x / pixel.actual.hits),
        float(pixel.actual.radiance.y / pixel.actual.hits),
        float(pixel.actual.radiance.z / pixel.actual.hits),
        (float)pixel.actual.hits / (float)pixel.actual.samples};
  } else {
    state->render[ij] = {
        0, 0, 0, (float)pixel.actual.hits / (float)pixel.actual.samples};
  }
  if (pixel.actual.hits) {
    state->odd_render[ij] = {float(pixel.odd.radiance.x / pixel.odd.hits),
        float(pixel.odd.radiance.y / pixel.odd.hits),
        float(pixel.odd.radiance.z / pixel.odd.hits),
        (float)pixel.odd.hits / (float)pixel.odd.samples};
  } else {
    state->odd_render[ij] = {
        0, 0, 0, (float)pixel.odd.hits / (float)pixel.odd.samples};
  }

  if (pixel.actual.samples < params.max_samples) {
    vec4f srgb     = rgb_to_srgb(state->render[ij]);
    vec4f srgb_odd = rgb_to_srgb(state->odd_render[ij]);

    double err_px = 0;
    double div    = sqrt(srgb.x + srgb.y + srgb.z);

    if (div >= 0.0001f) {
      err_px = ((double)abs(srgb.x - srgb_odd.x) +
                   (double)abs(srgb.y - srgb_odd.y) +
                   (double)abs(srgb.z - srgb_odd.z)) /
               (double)sqrt(double(srgb.x) + double(srgb.y) + double(srgb.z));
    } else {
      err_px = ((double)abs(srgb.x - srgb_odd.x) +
                   (double)abs(srgb.y - srgb_odd.y) +
                   (double)abs(srgb.z - srgb_odd.z)) /
               0.01;
    }

    pixel.q = -log2(err_px);
    if (pixel.q > 10) {
      pixel.q = 10;
    }
  } else {
    pixel.q = 10;
  }
}

  void collect_statistics(statistic & stat, const pathtrace_state* state) {
    auto size = state->render.imsize();

    int    pixels  = 0;
    float  min_q   = std::numeric_limits<float>::max();
    float  max_q   = -std::numeric_limits<float>::max();
    int    min_spp = std::numeric_limits<int>::max();
    double avg_spp = 0;
    int    max_spp = 0;

    for (auto i = 0; i < size.x; i++) {
      for (auto j = 0; j < size.y; j++) {
        auto& pixel = state->pixels[{i, j}];

        pixels++;

        if (pixel.q < min_q) {
          min_q = pixel.q;
        }

        if (pixel.q > max_q) {
          max_q = pixel.q;
        }

        if (pixel.actual.samples < min_spp) {
          min_spp = pixel.actual.samples;
        }

        if (pixel.actual.samples > max_spp) {
          max_spp = pixel.actual.samples;
        }
      }
    }

    avg_spp = double(state->sample_count) / double(pixels);

    stat.samples = state->sample_count;
    stat.pixels  = pixels;
    stat.min_q   = min_q;
    stat.max_q   = max_q;
    stat.min_spp = min_spp;
    stat.avg_spp = avg_spp;
    stat.max_spp = max_spp;

    static auto pad = [](const std::string& str, int n) -> std::string {
      return std::string(std::max(0, n - (int)str.size()), '0') + str;
    };

    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now() - state->start_time)
                       .count();
    auto mins  = pad(std::to_string(elapsed / 60000), 2);
    auto secs  = pad(std::to_string((elapsed % 60000) / 1000), 2);
    auto msecs = pad(std::to_string((elapsed % 60000) % 1000), 3);

    stat.stat_text = "    Current q: " + std::to_string(state->curr_q) + "\n" +
                     "      min_spp: " + std::to_string(stat.min_spp) + "\n" +
                     "      avg_spp: " + std::to_string(stat.avg_spp) + "\n" +
                     "      max_spp: " + std::to_string(stat.max_spp) + "\n" +
                     "sampling time: " + mins + ":" + secs + "." + msecs + "\n";
  }

}  // namespace yocto
