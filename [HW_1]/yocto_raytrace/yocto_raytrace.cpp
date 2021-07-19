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

#include "yocto_raytrace.h"

#include <yocto/yocto_color.h>
#include <yocto/yocto_geometry.h>
#include <yocto/yocto_parallel.h>
#include <yocto/yocto_shading.h>

// -----------------------------------------------------------------------------
// IMPLEMENTATION FOR SCENE EVALUATION
// -----------------------------------------------------------------------------
namespace yocto {

// Check texture size
static vec2i texture_size(const raytrace_texture* texture) {
  if (!texture->hdr.empty()) {
    return texture->hdr.imsize();
  } else if (!texture->ldr.empty()) {
    return texture->ldr.imsize();
  } else {
    return zero2i;
  }
}

// Evaluate a texture
static vec4f lookup_texture(const raytrace_texture* texture, const vec2i& ij,
    bool ldr_as_linear = false) {
  if (!texture->hdr.empty()) {
    return texture->hdr[ij];
  } else if (!texture->ldr.empty()) {
    return ldr_as_linear ? byte_to_float(texture->ldr[ij])
                         : srgb_to_rgb(byte_to_float(texture->ldr[ij]));
  } else {
    return {1, 1, 1, 1};
  }
}

// Evaluate a texture
static vec4f eval_texture(const raytrace_texture* texture, const vec2f& uv,
    bool ldr_as_linear = false, bool no_interpolation = false,
    bool clamp_to_edge = false) {
  // get texture
  if (!texture) return {1, 1, 1};

  // get yimg::image width/height
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

  // get yimg::image coordinates and residuals
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

// Generates a ray from a camera for yimg::image plane coordinate uv and
// the lens coordinates luv. YOUR CODE GOES HERE -----------------------
static ray3f eval_camera(const raytrace_camera* camera, const vec2f& image_uv) {
  auto e = zero3f;
  auto q = vec3f{camera->film.x * (0.5f - image_uv.x),
      camera->film.y * (image_uv.y - 0.5f), camera->lens};
  auto d   = normalize(-q - e);
  auto ray = ray3f{
      transform_point(camera->frame, e), transform_direction(camera->frame, d)};
  return ray;
}

// Eval position
static vec3f eval_position(
    const raytrace_shape* shape, int element, const vec2f& uv) {
  if (!shape->triangles.empty()) {
    auto t = shape->triangles[element];
    return interpolate_triangle(shape->positions[t.x], shape->positions[t.y],
        shape->positions[t.z], uv);
  }
  if (!shape->lines.empty()) {
    auto l = shape->lines[element];
    return interpolate_line(shape->positions[l.x], shape->positions[l.y], uv.x);
  }
  if (!shape->points.empty()) {
    auto p = shape->points[element];
    return shape->positions[p];
  }
  return zero3f;
}

// Shape element normal.  //YOUR CODE GOES HERE -----------------------
static vec3f eval_element_normal(const raytrace_shape* shape, int element) {
  if (!shape->triangles.empty()) {
    auto t = shape->triangles[element];
    return triangle_normal(
        shape->positions[t.x], shape->positions[t.y], shape->positions[t.z]);
  }
  if (!shape->lines.empty()) {
    auto l = shape->lines[element];
    return line_tangent(shape->positions[l.x], shape->positions[l.y]);
  }
  if (!shape->points.empty()) {
    auto p = shape->points[element];
    shape->positions[p];
  }
  return zero3f;
}

// Eval normal. YOUR CODE GOES HERE -----------------------
static vec3f eval_normal(
    const raytrace_shape* shape, int element, const vec2f& uv) {
  if (shape->normals.empty()) return eval_element_normal(shape, element);
  if (!shape->triangles.empty()) {
    auto t = shape->triangles[element];
    return normalize(interpolate_triangle(
        shape->normals[t.x], shape->normals[t.y], shape->normals[t.z], uv));
  }
  if (!shape->lines.empty()) {
    auto l = shape->lines[element];
    return normalize(
        interpolate_line(shape->normals[l.x], shape->normals[l.y], uv.x));
  }
  if (!shape->points.empty()) {
    auto p = shape->points[element];
    return normalize(shape->normals[p]);
  }
    return zero3f;
}

// Eval texcoord // YOUR CODE GOES HERE -----------------------
static vec2f eval_texcoord(
    const raytrace_shape* shape, int element, const vec2f& uv) {
  if (shape->texcoords.empty()) return uv;
  if (!shape->triangles.empty()) {
    auto t = shape->triangles[element];
    return interpolate_triangle(shape->texcoords[t.x], shape->texcoords[t.y],
        shape->texcoords[t.z], uv);
  }
  if (!shape->lines.empty()) {
    auto l = shape->lines[element];
    return interpolate_line(shape->texcoords[l.x], shape->texcoords[l.y], uv.x);
  }
  if (!shape->points.empty()) {
    auto p = shape->points[element];
    return shape->texcoords[p];
  }
  return zero2f;
}

// YOUR CODE GOES HERE -----------------------
// Evaluate all environment color.
static vec3f eval_environment(const raytrace_scene* scene, const ray3f& ray) {
  if (scene->environments.size() == 0) return zero3f;

  auto emission = zero3f;
  for (auto enviroment : scene->environments) {
    auto local_dir = transform_direction(inverse(enviroment->frame), ray.d);
    auto texcoord  = vec2f{atan2(local_dir.z, local_dir.x) / (2 * pif),
        acos(clamp(local_dir.y, -1., 1.)) / pif};
    if (texcoord.x < 0) texcoord.x += 1;
    emission += enviroment->emission *
                rgba_to_rgb(eval_texture(enviroment->emission_tex, texcoord));
  }
  return emission;
}

}  // namespace yocto

// -----------------------------------------------------------------------------
// IMPLEMENTATION FOR SHAPE/SCENE BVH
// -----------------------------------------------------------------------------
namespace yocto {

// primitive used to sort bvh entries
struct raytrace_bvh_primitive {
  bbox3f bbox      = invalidb3f;
  vec3f  center    = zero3f;
  int    primitive = 0;
};

// Splits a BVH node. Returns split position and axis.
static pair<int, int> split_middle(
    vector<raytrace_bvh_primitive>& primitives, int start, int end) {
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
static void build_bvh(vector<raytrace_bvh_node>& nodes,
    vector<raytrace_bvh_primitive>&              primitives) {
  // prepare to build nodes
  nodes.clear();
  nodes.reserve(primitives.size() * 2);

  // queue up first node
  auto queue = std::deque<vec3i>{{0, 0, (int)primitives.size()}};
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
      node.axis     = axis;
      node.num      = 2;
      node.start    = (int)nodes.size();
      nodes.emplace_back();
      nodes.emplace_back();
      queue.push_back({node.start + 0, start, mid});
      queue.push_back({node.start + 1, mid, end});
    } else {
      // Make a leaf node
      node.internal = false;
      node.num      = end - start;
      node.start    = start;
    }
  }

  // cleanup
  nodes.shrink_to_fit();
}

static void init_bvh(raytrace_shape* shape, const raytrace_params& params) {
  // build primitives
  auto primitives = vector<raytrace_bvh_primitive>{};
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
  }

  // build nodes
  if (shape->bvh) delete shape->bvh;
  shape->bvh = new raytrace_bvh_tree{};
  build_bvh(shape->bvh->nodes, primitives);

  // set bvh primitives
  shape->bvh->primitives.reserve(primitives.size());
  for (auto& primitive : primitives) {
    shape->bvh->primitives.push_back(primitive.primitive);
  }
}

void init_bvh(raytrace_scene* scene, const raytrace_params& params,
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
  auto primitives = vector<raytrace_bvh_primitive>{};
  auto object_id  = 0;
  for (auto instance : scene->instances) {
    auto& primitive = primitives.emplace_back();
    primitive.bbox  = instance->shape->bvh->nodes.empty()
                         ? invalidb3f
                         : transform_bbox(instance->frame,
                               instance->shape->bvh->nodes[0].bbox);
    primitive.center    = center(primitive.bbox);
    primitive.primitive = object_id++;
  }

  // build nodes
  if (scene->bvh) delete scene->bvh;
  scene->bvh = new raytrace_bvh_tree{};
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
static bool intersect_shape_bvh(raytrace_shape* shape, const ray3f& ray_,
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
    }

    // check for early exit
    if (find_any && hit) return hit;
  }

  return hit;
}

// Intersect ray with a bvh->
static bool intersect_scene_bvh(const raytrace_scene* scene, const ray3f& ray_,
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
static bool intersect_instance_bvh(const raytrace_instance* instance,
    const ray3f& ray, int& element, vec2f& uv, float& distance, bool find_any,
    bool non_rigid_frames) {
  auto inv_ray = transform_ray(inverse(instance->frame, non_rigid_frames), ray);
  return intersect_shape_bvh(
      instance->shape, inv_ray, element, uv, distance, find_any);
}

raytrace_intersection intersect_scene_bvh(const raytrace_scene* scene,
    const ray3f& ray, bool find_any, bool non_rigid_frames) {
  auto intersection = raytrace_intersection{};
  intersection.hit  = intersect_scene_bvh(scene, ray, intersection.instance,
      intersection.element, intersection.uv, intersection.distance, find_any,
      non_rigid_frames);
  return intersection;
}
raytrace_intersection intersect_instance_bvh(const raytrace_instance* instance,
    const ray3f& ray, bool find_any, bool non_rigid_frames) {
  auto intersection = raytrace_intersection{};
  intersection.hit = intersect_instance_bvh(instance, ray, intersection.element,
      intersection.uv, intersection.distance, find_any, non_rigid_frames);
  return intersection;
}

}  // namespace yocto

// -----------------------------------------------------------------------------
// IMPLEMENTATION FOR PATH TRACING
// -----------------------------------------------------------------------------
namespace yocto {

// Raytrace renderer.  // YOUR CODE GOES HERE -----------------------
static vec4f shade_raytrace(const raytrace_scene* scene, const ray3f& ray,
    int bounce, rng_state& rng, const raytrace_params& params) {
  auto isec = raytrace_intersection{};
  isec      = intersect_scene_bvh(scene, ray);
  if (isec.hit) {
    auto object = scene->instances[isec.instance];
    auto vec    = vec3f{0.04, 0.04, 0.04};

    //<compute position, normal, texcoord>
    auto normal = transform_direction(
        object->frame, eval_normal(object->shape, isec.element, isec.uv));
    auto outgoing = -ray.d;
    auto position = transform_point(
        object->frame, eval_position(object->shape, isec.element, isec.uv));

    auto texcoord = eval_texcoord(object->shape, isec.element, isec.uv);
    auto color    = rgb_to_rgba(object->material->color) *
                 eval_texture(object->material->color_tex, texcoord);
    auto color_rgb = rgba_to_rgb(color);

    //<compute material values accounting for textures> 
    auto roughness = object->material->roughness * object->material->roughness;
    auto opacity   = object->material->opacity *
                   eval_texture(object->material->opacity_tex, texcoord);

    if (!object->shape->lines.empty())
      normal = orthonormalize(outgoing, normal);
    if (dot(normal, ray.d) >= 0) normal = -normal;

    //<handle opacity>
    if (rand1f(rng) > opacity.x) {
      outgoing = -outgoing;
      return shade_raytrace(scene, ray3f{position + outgoing * 1e-2f, outgoing},
          bounce + 1, rng, params);// how prof said at lesson
    }

    auto radiance = rgb_to_rgba(object->material->emission);

    if (bounce >= params.bounces) return radiance;
    
    if (object->material->transmission) {// <handle polished dielectrics>
      if (rand1f(rng) < fresnel_schlick(vec, normal, outgoing).x) {
        auto incoming = reflect(outgoing, normal);
        radiance += shade_raytrace(
            scene, ray3f{position, incoming}, bounce + 1, rng, params);
      } else {
        auto incoming = -outgoing;
        radiance += color *
                   shade_raytrace(scene, ray3f{position, incoming}, bounce + 1,
                       rng, params);
      }
    } else if (object->material->metallic && !object->material->roughness) {// <handle polished metals>
      auto incoming = reflect(outgoing, normal);
      radiance += rgb_to_rgba(fresnel_schlick(color_rgb, normal, outgoing)) *
                  shade_raytrace(scene, ray3f{position, incoming}, bounce + 1,
                      rng, params);
    } else if (object->material->metallic && object->material->roughness) {// <handle rough metals>
      auto incoming = sample_hemisphere(normal, rand2f(rng));
      auto halfway  = normalize(outgoing + incoming);
      radiance +=
          rgb_to_rgba((2 * pif) *
                      fresnel_schlick(color_rgb, halfway, outgoing) *
                      microfacet_distribution(roughness, normal, halfway) *
                      microfacet_shadowing(
                          roughness, normal, halfway, outgoing, incoming) /
                      (4 * dot(normal, outgoing) * dot(normal, incoming))) *
          shade_raytrace(
              scene, ray3f{position, incoming}, bounce + 1, rng, params) *
          dot(normal, incoming);
    } else if (object->material->specular && object->material->roughness) { //<handle rough plastic>
      auto incoming = sample_hemisphere(normal, rand2f(rng));
      auto halfway  = normalize(outgoing + incoming);
      radiance +=
          rgb_to_rgba(
              (2 * pif) *
              (color_rgb / pi * (1 - fresnel_schlick(vec, halfway, outgoing)) +
                  fresnel_schlick(vec, halfway, outgoing) *
                      microfacet_distribution(roughness, normal, halfway) *
                      microfacet_shadowing(
                          roughness, normal, halfway, outgoing, incoming) /
                      (4 * dot(normal, outgoing) * dot(normal, incoming)))) *
          shade_raytrace(
              scene, ray3f{position, incoming}, bounce + 1, rng, params) *
          dot(normal, incoming);
    } else { // <handle diffuse>
      auto incoming = sample_hemisphere(normal, rand2f(rng));
      radiance += (2 * pif) * color / pif *
                  shade_raytrace(scene, ray3f{position, incoming}, bounce + 1,
                      rng, params) *
                  dot(normal, incoming);
    }
    return radiance;
  }
  return rgb_to_rgba(eval_environment(scene, ray));
}

static vec4f shade_refraction(const raytrace_scene* scene, const ray3f& ray,
    int bounce, rng_state& rng, const raytrace_params& params) {
  auto isec = raytrace_intersection{};
  isec      = intersect_scene_bvh(scene, ray);
  if (isec.hit) {
    auto object = scene->instances[isec.instance];
    auto vec    = vec3f{0.04, 0.04, 0.04};

    //<compute position, normal, texcoord>
    auto normal = transform_direction(
        object->frame, eval_normal(object->shape, isec.element, isec.uv));
    auto outgoing = -ray.d;
    auto position = transform_point(
        object->frame, eval_position(object->shape, isec.element, isec.uv));

    auto texcoord = eval_texcoord(object->shape, isec.element, isec.uv);
    auto color    = rgb_to_rgba(object->material->color) *
                 eval_texture(object->material->color_tex, texcoord);
    auto color_rgb = rgba_to_rgb(color);

    //<compute material values accounting for textures>
    auto roughness = object->material->roughness * object->material->roughness;
    auto opacity   = object->material->opacity *
                   eval_texture(object->material->opacity_tex, texcoord);

    if (!object->shape->lines.empty()) normal = orthonormalize(-ray.d, normal);
    if (dot(normal, ray.d) >= 0) normal = -normal;

    //<handle opacity>
    if (rand1f(rng) > opacity.x) {
      outgoing = -outgoing;
      return shade_raytrace(scene, ray3f{position + outgoing * 1e-2f, outgoing},
          bounce + 1, rng, params);  // how prof said at lesson
    }

    auto radiance = rgb_to_rgba(object->material->emission);

    if (bounce >= params.bounces) return radiance;

    if (object->material->transmission) {
      if (rand1f(rng) < fresnel_schlick(vec, normal, outgoing).x) {
        auto incoming = reflect(outgoing, normal);
        radiance += shade_raytrace(
            scene, ray3f{position, incoming}, bounce + 1, rng, params);
      } else {
        auto eta = reflectivity_to_eta(vec3f{0.04});
        if (dot(normal, outgoing) < 0) eta = -eta;
        auto inv_eta  = 1 / eta;
        auto incoming = refract(outgoing, normal, inv_eta.x);
        radiance += color * shade_raytrace(scene, ray3f{position, incoming},
                                bounce + 1, rng, params);
      }
    } else if (object->material->metallic &&
               !object->material->roughness) {  // <handle polished metals>
      auto incoming = reflect(outgoing, normal);
      radiance += rgb_to_rgba(fresnel_schlick(color_rgb, normal, outgoing)) *
                  shade_raytrace(scene, ray3f{position, incoming}, bounce + 1,
                      rng, params);
    } else if (object->material->metallic &&
               object->material->roughness) {  // <handle rough metals>
      auto incoming = sample_hemisphere(normal, rand2f(rng));
      auto halfway  = normalize(outgoing + incoming);
      radiance +=
          rgb_to_rgba((2 * pif) *
                      fresnel_schlick(color_rgb, halfway, outgoing) *
                      microfacet_distribution(roughness, normal, halfway) *
                      microfacet_shadowing(
                          roughness, normal, halfway, outgoing, incoming) /
                      (4 * dot(normal, outgoing) * dot(normal, incoming))) *
          shade_raytrace(
              scene, ray3f{position, incoming}, bounce + 1, rng, params) *
          dot(normal, incoming);
    } else if (object->material->specular &&
               object->material->roughness) {  //<handle rough plastic>
      auto incoming = sample_hemisphere(normal, rand2f(rng));
      auto halfway  = normalize(outgoing + incoming);
      radiance +=
          rgb_to_rgba(
              (2 * pif) *
              (color_rgb / pi * (1 - fresnel_schlick(vec, halfway, outgoing)) +
                  fresnel_schlick(vec, halfway, outgoing) *
                      microfacet_distribution(roughness, normal, halfway) *
                      microfacet_shadowing(
                          roughness, normal, halfway, outgoing, incoming) /
                      (4 * dot(normal, outgoing) * dot(normal, incoming)))) *
          shade_raytrace(
              scene, ray3f{position, incoming}, bounce + 1, rng, params) *
          dot(normal, incoming);
    } else {  // <handle diffuse>
      auto incoming = sample_hemisphere(normal, rand2f(rng));
      radiance += (2 * pif) * color / pif *
                  shade_raytrace(scene, ray3f{position, incoming}, bounce + 1,
                      rng, params) *
                  dot(normal, incoming);
    }
    return radiance;
  } else {
    return rgb_to_rgba(eval_environment(scene, ray));
  }
  return zero4f;
}

// Eyelight for quick previewing.  // YOUR CODE GOES HERE
// -----------------------
static vec4f shade_eyelight(const raytrace_scene* scene, const ray3f& ray,
    int bounce, rng_state& rng, const raytrace_params& params) {
  auto isec = raytrace_intersection{};
  isec      = intersect_scene_bvh(scene, ray);
  if (isec.hit) {
    auto object = scene->instances[isec.instance];
    auto normal = transform_direction(
        object->frame, eval_normal(object->shape, isec.element, isec.uv));
    return rgb_to_rgba(object->material->color * dot(normal, -ray.d));
  }
  return zero4f;
}

// YOUR CODE GOES HERE -----------------------
static vec4f shade_normal(const raytrace_scene* scene, const ray3f& ray,
    int bounce, rng_state& rng, const raytrace_params& params) {
  auto isec = raytrace_intersection{};
  isec      = intersect_scene_bvh(scene, ray);
  if (isec.hit) {
    auto object = scene->instances[isec.instance];
    auto normal = eval_normal(object->shape, isec.element, isec.uv);
    return rgb_to_rgba((normal * 0.5) + 0.5);
  }
  return one4f;
}

// YOUR CODE GOES HERE -----------------------
static vec4f shade_texcoord(const raytrace_scene* scene, const ray3f& ray,
    int bounce, rng_state& rng, const raytrace_params& params) {
  auto isec = raytrace_intersection();
  isec      = intersect_scene_bvh(scene, ray);
  if (isec.hit) {
    auto object    = scene->instances[isec.instance];
    auto textcoord = eval_texcoord(object->shape, isec.element, isec.uv);
    return vec4f{fmod(textcoord.x, 1), fmod(textcoord.y, 1), 0, 1};
  }
  return zero4f;
}

static vec4f shade_color(const raytrace_scene* scene, const ray3f& ray,
    int bounce, rng_state& rng, const raytrace_params& params) {
  auto isec = raytrace_intersection{};
  isec      = intersect_scene_bvh(scene, ray);
  if (isec.hit) {
    //auto object = scene->instances[isec.instance];
    auto color  = scene->materials[isec.instance]->color;
    return rgb_to_rgba(color);
    // * eval_texture(object->material->color_tex, eval_texcoord(object->shape, isec.element, isec.uv));
  }
  return zero4f;
}

static vec4f shade_my1(const raytrace_scene* scene, const ray3f& ray,
    int bounce, rng_state& rng, const raytrace_params& params) {
  auto isec = raytrace_intersection{};
  isec      = intersect_scene_bvh(scene, ray);
  if (isec.hit) {
    auto object = scene->instances[isec.instance];
    auto color  = shade_raytrace(scene, ray, bounce, rng, params);

    if (color.x + color.y + color.z >= 2.5f) {
      color = vec4f{1, 1, 1, 1};
    } else if (color.x + color.y + color.z >= 1.5f) {
      auto prop = color.x + color.y + color.z;
      color     = color * 2.f / prop;
    } else {
      auto prop = color.x + color.y + color.z;
      color     = color * 1.f / prop;
    }
    return color;
  }
  return zero4f;
}

static vec4f shade_my2(const raytrace_scene* scene, const ray3f& ray,
    int bounce, rng_state& rng, const raytrace_params& params) {
  auto isec = raytrace_intersection{};
  isec      = intersect_scene_bvh(scene, ray);
  if (isec.hit) {
    auto object = scene->instances[isec.instance];
    auto color  = shade_raytrace(scene, ray, bounce, rng, params);
    auto normal = transform_direction(
        object->frame, eval_normal(object->shape, isec.element, isec.uv));

    if (normal.y > 0) {
      auto prop = color.x + color.y + color.z;
      color     = color * (1 - normal.y) / prop * vec4f{0.5, 0.5, 1, 1};
    } else if (normal.y < 0) {
      auto prop = color.x + color.y + color.z;
      color     = color * (-normal.y) / prop * vec4f{1, 0.5, 0.5, 1};
    } else {
      color   = zero4f;
    }
    return color;
  }
  return zero4f;
}

static vec4f shade_my3(const raytrace_scene* scene, const ray3f& ray,
    int bounce, rng_state& rng, const raytrace_params& params) {
  auto isec = raytrace_intersection{};
  isec      = intersect_scene_bvh(scene, ray);
  if (isec.hit) {
    auto object = scene->instances[isec.instance];
    auto color  = rgb_to_rgba(object->material->color) *
                 eval_texture(object->material->color_tex,
                     eval_texcoord(object->shape, isec.element, isec.uv));
    auto normal = transform_direction(
        object->frame, eval_normal(object->shape, isec.element, isec.uv));

    
    auto val = dot(normal, -ray.d);
    if (val > 0.85f) {
      color *= {1.2, 1.2, 1.2, 1};
    } else if (val > 0.5f) {
      color *= {0.3, 0.3, 0.3, 1};
    } else {
      color *= {0.11, 0.11, 0.11, 1};
    }
    return color;
  }
  return zero4f;
}

static vec4f shade_my4(const raytrace_scene* scene, const ray3f& ray,
    int bounce, rng_state& rng, const raytrace_params& params) {
  auto isec = raytrace_intersection{};
  isec      = intersect_scene_bvh(scene, ray);
  if (isec.hit) {
    auto object = scene->instances[isec.instance];
    auto color  = shade_raytrace(scene, ray, bounce, rng, params);

    if (color.x > 0.5f && color.y < 0.5f && color.z < 0.5f) {
      color = color * vec4f{1.2, 0, 0, 1};
    } else {
      color = vec4f{0.04, 0.04, 0.04, 1} * gray_to_rgba(rgba_to_gray(color));
    }
    return color;
  }
  return zero4f;
}

static vec4f shade_my5(const raytrace_scene* scene, const ray3f& ray,
    int bounce, rng_state& rng, const raytrace_params& params) {

  return zero4f;
}

static vec4f shade_my6(const raytrace_scene* scene, const ray3f& ray,
    int bounce, rng_state& rng, const raytrace_params& params) {

  return zero4f;
}

static vec4f shade_my7(const raytrace_scene* scene, const ray3f& ray,
    int bounce, rng_state& rng, const raytrace_params& params) {

  return zero4f;
}

// Trace a single ray from the camera using the given algorithm.
using raytrace_shader_func = vec4f (*)(const raytrace_scene* scene,
    const ray3f& ray, int bounce, rng_state& rng,
    const raytrace_params& params);
static raytrace_shader_func get_shader(const raytrace_params& params) {
  switch (params.shader) {
    case raytrace_shader_type::raytrace: return shade_raytrace;
    case raytrace_shader_type::refraction: return shade_refraction;
    case raytrace_shader_type::eyelight: return shade_eyelight;
    case raytrace_shader_type::normal: return shade_normal;
    case raytrace_shader_type::texcoord: return shade_texcoord;
    case raytrace_shader_type::color: return shade_color;
    case raytrace_shader_type::my1: return shade_my1;
    case raytrace_shader_type::my2: return shade_my2;
    case raytrace_shader_type::my3: return shade_my3;
    case raytrace_shader_type::my4: return shade_my4;
    case raytrace_shader_type::my5: return shade_my5;
    case raytrace_shader_type::my6: return shade_my6;
    case raytrace_shader_type::my7: return shade_my7;
    default: {
      throw std::runtime_error("sampler unknown");
      return nullptr;
    }
  }
}

// Trace a block of samples  // YOUR CODE GOES HERE -----------------------
void render_sample(raytrace_state* state, const raytrace_scene* scene,
    const raytrace_camera* camera, const vec2i& ij,
    const raytrace_params& params) {

  auto shade = get_shader(params);
  auto size  = state->render.imsize();
  auto puv   = rand2f(state->rngs[ij]);
  auto uv    = vec2f{(ij.x + puv.x) / size.x, (ij.y + puv.y) / size.y};
  // auto uv    = vec2f{(i + 0.5f) / size.x, (j + 0.5f) / size.y};
  auto ray   = eval_camera(camera, uv);
  state->accumulation[ij] += clamp(
      shade(scene, ray, 0, state->rngs[ij], params), 0., params.clamp);
  state->samples[ij]++;
  state->render[ij] = state->accumulation[ij] / (state->samples[ij] + 1);
}

// Init a sequence of random number generators.
void init_state(raytrace_state* state, const raytrace_scene* scene,
    const raytrace_camera* camera, const raytrace_params& params) {
  auto image_size =
      (camera->film.x > camera->film.y)
          ? vec2i{params.resolution,
                (int)round(params.resolution * camera->film.y / camera->film.x)}
          : vec2i{
                (int)round(params.resolution * camera->film.x / camera->film.y),
                params.resolution};
  state->render.assign(image_size, zero4f);
  state->accumulation.assign(image_size, zero4f);
  state->samples.assign(image_size, 0);
  state->rngs.assign(image_size, {});
  auto init_rng = make_rng(1301081);
  for (auto& rng : state->rngs) {
    rng = make_rng(params.seed, rand1i(init_rng, 1 << 31) / 2 + 1);
  }
}

// Progressively compute an image by calling trace_samples multiple times.
void render_samples(raytrace_state* state, const raytrace_scene* scene,
    const raytrace_camera* camera, const raytrace_params& params) {
  auto size = state->render.imsize();
  if (params.noparallel) {
    for (auto j = 0; j < size.y; j++) {
      for (auto i = 0; i < size.x; i++) {
        render_sample(state, scene, camera, {i, j}, params);
      }
    }
  } else {
    parallel_for(size.x, size.y, [&](auto i = 0, auto j = 0) {
      render_sample(state, scene, camera, {i, j}, params);
    });
  }
}

}  // namespace yocto

// -----------------------------------------------------------------------------
// SCENE CREATION
// -----------------------------------------------------------------------------
namespace yocto {

// cleanup
raytrace_shape::~raytrace_shape() {
  if (bvh) delete bvh;
}

// cleanup
raytrace_scene::~raytrace_scene() {
  if (bvh) delete bvh;
  for (auto camera : cameras) delete camera;
  for (auto instance : instances) delete instance;
  for (auto shape : shapes) delete shape;
  for (auto material : materials) delete material;
  for (auto texture : textures) delete texture;
  for (auto environment : environments) delete environment;
}

// Add element
raytrace_camera* add_camera(raytrace_scene* scene) {
  return scene->cameras.emplace_back(new raytrace_camera{});
}
raytrace_texture* add_texture(raytrace_scene* scene) {
  return scene->textures.emplace_back(new raytrace_texture{});
}
raytrace_shape* add_shape(raytrace_scene* scene) {
  return scene->shapes.emplace_back(new raytrace_shape{});
}
raytrace_material* add_material(raytrace_scene* scene) {
  return scene->materials.emplace_back(new raytrace_material{});
}
raytrace_instance* add_instance(raytrace_scene* scene) {
  return scene->instances.emplace_back(new raytrace_instance{});
}
raytrace_environment* add_environment(raytrace_scene* scene) {
  return scene->environments.emplace_back(new raytrace_environment{});
}

// Set cameras
void set_frame(raytrace_camera* camera, const frame3f& frame) {
  camera->frame = frame;
}
void set_lens(raytrace_camera* camera, float lens, float aspect, float film) {
  camera->lens = lens;
  camera->film = aspect >= 1 ? vec2f{film, film / aspect}
                             : vec2f{film * aspect, film};
}
void set_focus(raytrace_camera* camera, float aperture, float focus) {
  camera->aperture = aperture;
  camera->focus    = focus;
}

// Add texture
void set_texture(raytrace_texture* texture, const image<vec4b>& img) {
  texture->ldr = img;
  texture->hdr = {};
}
void set_texture(raytrace_texture* texture, const image<vec4f>& img) {
  texture->ldr = {};
  texture->hdr = img;
}

// Add shape
void set_points(raytrace_shape* shape, const vector<int>& points) {
  shape->points = points;
}
void set_lines(raytrace_shape* shape, const vector<vec2i>& lines) {
  shape->lines = lines;
}
void set_triangles(raytrace_shape* shape, const vector<vec3i>& triangles) {
  shape->triangles = triangles;
}
void set_positions(raytrace_shape* shape, const vector<vec3f>& positions) {
  shape->positions = positions;
}
void set_normals(raytrace_shape* shape, const vector<vec3f>& normals) {
  shape->normals = normals;
}
void set_texcoords(raytrace_shape* shape, const vector<vec2f>& texcoords) {
  shape->texcoords = texcoords;
}
void set_radius(raytrace_shape* shape, const vector<float>& radius) {
  shape->radius = radius;
}

// Add instance
void set_frame(raytrace_instance* instance, const frame3f& frame) {
  instance->frame = frame;
}
void set_shape(raytrace_instance* instance, raytrace_shape* shape) {
  instance->shape = shape;
}
void set_material(raytrace_instance* instance, raytrace_material* material) {
  instance->material = material;
}

// Add material
void set_emission(raytrace_material* material, const vec3f& emission,
    raytrace_texture* emission_tex) {
  material->emission     = emission;
  material->emission_tex = emission_tex;
}
void set_color(raytrace_material* material, const vec3f& color,
    raytrace_texture* color_tex) {
  material->color     = color;
  material->color_tex = color_tex;
}
void set_specular(raytrace_material* material, float specular,
    raytrace_texture* specular_tex) {
  material->specular     = specular;
  material->specular_tex = specular_tex;
}
void set_metallic(raytrace_material* material, float metallic,
    raytrace_texture* metallic_tex) {
  material->metallic     = metallic;
  material->metallic_tex = metallic_tex;
}
void set_ior(raytrace_material* material, float ior) { material->ior = ior; }
void set_transmission(raytrace_material* material, float transmission,
    bool thin, float trdepth, raytrace_texture* transmission_tex) {
  material->transmission     = transmission;
  material->thin             = thin;
  material->trdepth          = trdepth;
  material->transmission_tex = transmission_tex;
}
void set_thin(raytrace_material* material, bool thin) { material->thin = thin; }
void set_roughness(raytrace_material* material, float roughness,
    raytrace_texture* roughness_tex) {
  material->roughness     = roughness;
  material->roughness_tex = roughness_tex;
}
void set_opacity(
    raytrace_material* material, float opacity, raytrace_texture* opacity_tex) {
  material->opacity     = opacity;
  material->opacity_tex = opacity_tex;
}
void set_scattering(raytrace_material* material, const vec3f& scattering,
    float scanisotropy, raytrace_texture* scattering_tex) {
  material->scattering     = scattering;
  material->scanisotropy   = scanisotropy;
  material->scattering_tex = scattering_tex;
}

// Add environment
void set_frame(raytrace_environment* environment, const frame3f& frame) {
  environment->frame = frame;
}
void set_emission(raytrace_environment* environment, const vec3f& emission,
    raytrace_texture* emission_tex) {
  environment->emission     = emission;
  environment->emission_tex = emission_tex;
}

}  // namespace yocto
