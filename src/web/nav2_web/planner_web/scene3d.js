'use strict';

/* Offline WebGL scene renderer. No CDN or robot-side graphics dependency. */
(function createScanPlannerScene() {
  const canvas = document.getElementById('sceneCanvas');
  if (!canvas) return;

  const defaultCamera = () => ({
    target: [0, 0, 0.4],
    distance: 14,
    yaw: -0.82,
    pitch: 0.62,
    follow: false,
  });

  const gl = canvas.getContext('webgl2', {
    alpha: false,
    antialias: true,
    depth: true,
    powerPreference: 'high-performance',
  }) || canvas.getContext('webgl', {
    alpha: false,
    antialias: true,
    depth: true,
    powerPreference: 'high-performance',
  });

  const scene = {
    enabled: true,
    connected: false,
    fixedFrame: 'map',
    objects: new Map(),
    markers: new Map(),
    poses: new Map(),
    transforms: new Map(),
    layers: {
      traversable: true,
      registered: true,
      global_map: true,
      occupancy: true,
      inflated: false,
      planning: true,
      robot: true,
      tf: false,
    },
    layerStatus: {},
    trail: [],
    camera: defaultCamera(),
    interactionMode: 'orbit',
    pointers: new Map(),
    gesture: null,
    primaryGesture: null,
    pendingGoal: null,
    routeWaypoints: [],
    bodyHeight: 0.4,
    activeProfile: null,
    pctMode: false,
    frameCount: 0,
    fps: 0,
    fpsTime: performance.now(),
    readoutTime: 0,
    dataReady: false,
    autoFitDone: false,
  };

  const byId = (id) => document.getElementById(id);
  const identity = () => new Float32Array([
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1,
  ]);

  function multiply(a, b) {
    const result = new Float32Array(16);
    for (let column = 0; column < 4; column += 1) {
      for (let row = 0; row < 4; row += 1) {
        let value = 0;
        for (let index = 0; index < 4; index += 1) {
          value += a[index * 4 + row] * b[column * 4 + index];
        }
        result[column * 4 + row] = value;
      }
    }
    return result;
  }

  function poseMatrix(position, quaternion) {
    const p = position || [0, 0, 0];
    let [x, y, z, w] = quaternion || [0, 0, 0, 1];
    const length = Math.hypot(x, y, z, w) || 1;
    x /= length; y /= length; z /= length; w /= length;
    const xx = x * x; const yy = y * y; const zz = z * z;
    const xy = x * y; const xz = x * z; const yz = y * z;
    const wx = w * x; const wy = w * y; const wz = w * z;
    return new Float32Array([
      1 - 2 * (yy + zz), 2 * (xy + wz), 2 * (xz - wy), 0,
      2 * (xy - wz), 1 - 2 * (xx + zz), 2 * (yz + wx), 0,
      2 * (xz + wy), 2 * (yz - wx), 1 - 2 * (xx + yy), 0,
      Number(p[0]) || 0, Number(p[1]) || 0, Number(p[2]) || 0, 1,
    ]);
  }

  function transformPoint(matrix, point) {
    const x = point[0]; const y = point[1]; const z = point[2];
    return [
      matrix[0] * x + matrix[4] * y + matrix[8] * z + matrix[12],
      matrix[1] * x + matrix[5] * y + matrix[9] * z + matrix[13],
      matrix[2] * x + matrix[6] * y + matrix[10] * z + matrix[14],
    ];
  }

  function perspective(fov, aspect, near, far) {
    const f = 1 / Math.tan(fov / 2);
    const range = 1 / (near - far);
    return new Float32Array([
      f / aspect, 0, 0, 0,
      0, f, 0, 0,
      0, 0, (far + near) * range, -1,
      0, 0, 2 * far * near * range, 0,
    ]);
  }

  function normalize(vector) {
    const length = Math.hypot(vector[0], vector[1], vector[2]) || 1;
    return [vector[0] / length, vector[1] / length, vector[2] / length];
  }

  function cross(a, b) {
    return [
      a[1] * b[2] - a[2] * b[1],
      a[2] * b[0] - a[0] * b[2],
      a[0] * b[1] - a[1] * b[0],
    ];
  }

  function lookAt(eye, center, up) {
    const z = normalize([
      eye[0] - center[0], eye[1] - center[1], eye[2] - center[2],
    ]);
    const x = normalize(cross(up, z));
    const y = cross(z, x);
    return new Float32Array([
      x[0], y[0], z[0], 0,
      x[1], y[1], z[1], 0,
      x[2], y[2], z[2], 0,
      -(x[0] * eye[0] + x[1] * eye[1] + x[2] * eye[2]),
      -(y[0] * eye[0] + y[1] * eye[1] + y[2] * eye[2]),
      -(z[0] * eye[0] + z[1] * eye[1] + z[2] * eye[2]),
      1,
    ]);
  }

  function resolveFrame(frame, visited = new Set()) {
    const cleanFrame = String(frame || scene.fixedFrame).replace(/^\//, '');
    if (!cleanFrame || cleanFrame === scene.fixedFrame) return identity();
    if (visited.has(cleanFrame)) return null;
    visited.add(cleanFrame);
    const edge = scene.transforms.get(cleanFrame);
    if (!edge) return null;
    const parent = resolveFrame(edge.parent, visited);
    if (!parent) return null;
    return multiply(parent, edge.matrix);
  }

  function compileShader(type, source) {
    const shader = gl.createShader(type);
    gl.shaderSource(shader, source);
    gl.compileShader(shader);
    if (!gl.getShaderParameter(shader, gl.COMPILE_STATUS)) {
      const message = gl.getShaderInfoLog(shader);
      gl.deleteShader(shader);
      throw new Error(message);
    }
    return shader;
  }

  function createProgram() {
    const vertex = compileShader(gl.VERTEX_SHADER, `
      attribute vec3 aPosition;
      uniform mat4 uViewProjection;
      uniform mat4 uModel;
      uniform float uPointSize;
      varying float vHeight;
      void main() {
        vec4 world = uModel * vec4(aPosition, 1.0);
        vHeight = world.z;
        gl_Position = uViewProjection * world;
        gl_PointSize = uPointSize;
      }
    `);
    const fragment = compileShader(gl.FRAGMENT_SHADER, `
      precision mediump float;
      uniform vec4 uColor;
      uniform vec2 uHeightRange;
      uniform float uColorMode;
      uniform float uRoundPoint;
      varying float vHeight;
      vec3 turbo(float x) {
        x = clamp(x, 0.0, 1.0);
        vec3 c0 = vec3(0.12, 0.20, 0.72);
        vec3 c1 = vec3(0.04, 0.86, 0.84);
        vec3 c2 = vec3(0.96, 0.88, 0.18);
        vec3 c3 = vec3(0.96, 0.16, 0.12);
        if (x < 0.34) return mix(c0, c1, x / 0.34);
        if (x < 0.68) return mix(c1, c2, (x - 0.34) / 0.34);
        return mix(c2, c3, (x - 0.68) / 0.32);
      }
      void main() {
        if (uRoundPoint > 0.5) {
          vec2 p = gl_PointCoord * 2.0 - 1.0;
          if (dot(p, p) > 1.0) discard;
        }
        vec3 color = uColor.rgb;
        if (uColorMode > 0.5) {
          float span = max(0.05, uHeightRange.y - uHeightRange.x);
          color = turbo((vHeight - uHeightRange.x) / span);
        }
        gl_FragColor = vec4(color, uColor.a);
      }
    `);
    const program = gl.createProgram();
    gl.attachShader(program, vertex);
    gl.attachShader(program, fragment);
    gl.linkProgram(program);
    if (!gl.getProgramParameter(program, gl.LINK_STATUS)) {
      throw new Error(gl.getProgramInfoLog(program));
    }
    return {
      program,
      position: gl.getAttribLocation(program, 'aPosition'),
      viewProjection: gl.getUniformLocation(program, 'uViewProjection'),
      model: gl.getUniformLocation(program, 'uModel'),
      pointSize: gl.getUniformLocation(program, 'uPointSize'),
      color: gl.getUniformLocation(program, 'uColor'),
      heightRange: gl.getUniformLocation(program, 'uHeightRange'),
      colorMode: gl.getUniformLocation(program, 'uColorMode'),
      roundPoint: gl.getUniformLocation(program, 'uRoundPoint'),
    };
  }

  let renderer = null;
  if (gl) {
    try {
      renderer = createProgram();
      gl.enable(gl.DEPTH_TEST);
      gl.depthFunc(gl.LEQUAL);
      gl.enable(gl.BLEND);
      gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA);
      gl.clearColor(0.018, 0.043, 0.064, 1);
    } catch (error) {
      console.error('WebGL 初始化失败', error);
    }
  }

  function decodeFloat32(encoded) {
    const binary = atob(encoded || '');
    const bytes = new Uint8Array(binary.length);
    for (let index = 0; index < binary.length; index += 1) {
      bytes[index] = binary.charCodeAt(index);
    }
    return new Float32Array(bytes.buffer);
  }

  function flattenPoints(points) {
    const values = new Float32Array((points || []).length * 3);
    (points || []).forEach((point, index) => {
      values[index * 3] = Number(point[0]) || 0;
      values[index * 3 + 1] = Number(point[1]) || 0;
      values[index * 3 + 2] = Number(point[2]) || 0;
    });
    return values;
  }

  function uploadObject(key, object) {
    if (!gl || !renderer) return;
    const previous = scene.objects.get(key);
    if (previous?.buffer) gl.deleteBuffer(previous.buffer);
    const buffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, buffer);
    gl.bufferData(gl.ARRAY_BUFFER, object.positions, gl.STATIC_DRAW);
    scene.objects.set(key, {
      ...object,
      key,
      buffer,
      count: Math.floor(object.positions.length / 3),
    });
    scene.dataReady = true;
    byId('sceneEmpty')?.classList.add('hidden');
  }

  function removeObject(key) {
    const object = scene.objects.get(key);
    if (object?.buffer && gl) gl.deleteBuffer(object.buffer);
    scene.objects.delete(key);
  }

  function cloudMessage(message) {
    const positions = decodeFloat32(message.xyz);
    uploadObject(`cloud:${message.layer}`, {
      layer: message.layer,
      frame: message.frame_id,
      positions,
      primitive: 'points',
      color: message.color || [0.4, 0.9, 1, 1],
      colorMode: message.color_mode === 'height',
      pointSize: Number(message.point_size) || 2,
      heightRange: [Number(message.min_z) || 0, Number(message.max_z) || 1],
    });
    scene.layerStatus[message.layer] = {
      count: positions.length / 3,
      age: 0,
    };
    // A prior-map PCD is often tens of metres away from the default camera
    // target. Fit exactly once after the first real cloud arrives, then leave
    // all subsequent camera movement under the operator's control.
    if (!scene.autoFitDone && positions.length >= 3) {
      scene.autoFitDone = true;
      requestAnimationFrame(fitScene);
    }
  }

  function markerMessage(message) {
    const markerKey = `marker:${message.key}`;
    if (message.action === 3) {
      [...scene.objects.keys()].forEach((key) => {
        if (key.startsWith(`marker:${message.topic}:`)) removeObject(key);
      });
      return;
    }
    if (message.action === 2) {
      removeObject(markerKey);
      return;
    }
    let points = message.points || [];
    let primitive = 'points';
    if (message.marker_type === 4) primitive = 'line_strip';
    else if (message.marker_type === 5) primitive = 'lines';
    else if (message.marker_type === 0 && points.length >= 2) primitive = 'lines';
    if (points.length === 0) points = [[0, 0, 0]];
    const pose = message.pose || {};
    uploadObject(markerKey, {
      layer: 'planning',
      frame: message.frame_id,
      pose: poseMatrix(pose.position, pose.orientation),
      positions: flattenPoints(points),
      primitive,
      color: message.color?.[3] > 0 ? message.color : [1, 0.3, 0.2, 1],
      colorMode: false,
      pointSize: Math.max(4, (Number(message.scale?.[0]) || 0.08) * 30),
      heightRange: [0, 1],
    });
  }

  function pathMessage(message) {
    uploadObject(`path:${message.topic}`, {
      layer: 'planning',
      frame: message.frame_id,
      positions: flattenPoints(message.points),
      primitive: 'line_strip',
      color: message.color || [0.2, 1, 0.5, 1],
      colorMode: false,
      pointSize: Number(message.line_width) || 2,
      heightRange: [0, 1],
    });
  }

  function poseMessage(message) {
    const pose = message.pose || {};
    const entry = {
      ...message,
      matrix: poseMatrix(pose.position, pose.orientation),
    };
    scene.poses.set(message.layer, entry);
    if (message.child_frame_id) {
      scene.transforms.set(message.child_frame_id.replace(/^\//, ''), {
        parent: String(message.frame_id || scene.fixedFrame).replace(/^\//, ''),
        matrix: entry.matrix,
      });
    }
    if (message.layer === 'body_pose') {
      const frameMatrix = resolveFrame(message.frame_id) || identity();
      const world = transformPoint(multiply(frameMatrix, entry.matrix), [0, 0, 0]);
      const last = scene.trail[scene.trail.length - 1];
      if (!last || Math.hypot(world[0] - last[0], world[1] - last[1], world[2] - last[2]) > 0.04) {
        scene.trail.push(world);
        if (scene.trail.length > 3000) scene.trail.splice(0, 500);
      }
    }
    scene.dataReady = true;
    byId('sceneEmpty')?.classList.add('hidden');
  }

  function tfMessage(message) {
    (message.transforms || []).forEach((transform) => {
      const child = String(transform.child || '').replace(/^\//, '');
      const parent = String(transform.parent || '').replace(/^\//, '');
      if (!child || !parent) return;
      scene.transforms.set(child, {
        parent,
        matrix: poseMatrix(transform.translation, transform.rotation),
        static: Boolean(message.static),
      });
    });
  }

  function statusMessage(message) {
    scene.layerStatus = { ...scene.layerStatus, ...(message.layers || {}) };
    const ids = {
      traversable: 'layerTraversable',
      registered: 'layerRegistered',
      global_map: 'layerGlobalMap',
      occupancy: 'layerOccupancy',
      inflated: 'layerInflated',
      planning: 'layerPlanning',
      robot: 'layerRobot',
      tf: 'layerTf',
    };
    Object.entries(ids).forEach(([layer, id]) => {
      const element = byId(id);
      if (!element) return;
      const status = scene.layerStatus[layer];
      if (!status) {
        element.textContent = layer === 'tf'
          ? `${message.transform_count || 0} 帧` : '等待';
        element.classList.toggle('live', layer === 'tf' && message.transform_count > 0);
        return;
      }
      const age = Number(status.age);
      const fresh = Number.isFinite(age) && age < 2.5;
      const count = Number(status.count) || 0;
      element.textContent = count > 1
        ? `${count.toLocaleString()} · ${age.toFixed(1)}s`
        : `${age.toFixed(1)}s`;
      element.classList.toggle('live', fresh);
    });
  }

  function handleMessage(message) {
    switch (message.type) {
      case 'scene_config':
        scene.fixedFrame = String(message.fixed_frame || 'map').replace(/^\//, '');
        if (byId('sceneFixedFrame')) {
          byId('sceneFixedFrame').textContent = `Fixed: ${scene.fixedFrame}`;
        }
        break;
      case 'scene_pointcloud': cloudMessage(message); break;
      case 'scene_marker': markerMessage(message); break;
      case 'scene_path': pathMessage(message); break;
      case 'scene_pose': poseMessage(message); break;
      case 'scene_tf': tfMessage(message); break;
      case 'scene_status': statusMessage(message); break;
      default: return false;
    }
    return true;
  }

  function createStaticObject(positions, color, primitive = 'lines') {
    if (!gl || !renderer) return null;
    const buffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, buffer);
    gl.bufferData(gl.ARRAY_BUFFER, positions, gl.STATIC_DRAW);
    return {
      buffer,
      positions,
      count: positions.length / 3,
      color,
      primitive,
      pointSize: 2,
      colorMode: false,
      heightRange: [0, 1],
      frame: scene.fixedFrame,
    };
  }

  function gridVertices() {
    const values = [];
    const extent = 50;
    for (let index = -extent; index <= extent; index += 1) {
      values.push(-extent, index, 0, extent, index, 0);
      values.push(index, -extent, 0, index, extent, 0);
    }
    return new Float32Array(values);
  }

  const grid = renderer
    ? createStaticObject(gridVertices(), [0.22, 0.39, 0.48, 0.22]) : null;
  const axis = renderer ? createStaticObject(new Float32Array([
    0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1,
  ]), [0.55, 0.7, 0.78, 0.8]) : null;

  const dogVertices = new Float32Array([
    -0.38, -0.16, -0.11, 0.38, -0.16, -0.11,
    0.38, -0.16, -0.11, 0.38, 0.16, -0.11,
    0.38, 0.16, -0.11, -0.38, 0.16, -0.11,
    -0.38, 0.16, -0.11, -0.38, -0.16, -0.11,
    -0.38, -0.16, 0.11, 0.38, -0.16, 0.11,
    0.38, -0.16, 0.11, 0.38, 0.16, 0.11,
    0.38, 0.16, 0.11, -0.38, 0.16, 0.11,
    -0.38, 0.16, 0.11, -0.38, -0.16, 0.11,
    -0.38, -0.16, -0.11, -0.38, -0.16, 0.11,
    0.38, -0.16, -0.11, 0.38, -0.16, 0.11,
    0.38, 0.16, -0.11, 0.38, 0.16, 0.11,
    -0.38, 0.16, -0.11, -0.38, 0.16, 0.11,
    0.28, -0.14, -0.08, 0.32, -0.19, -0.42,
    0.32, -0.19, -0.42, 0.36, -0.19, -0.62,
    0.28, 0.14, -0.08, 0.32, 0.19, -0.42,
    0.32, 0.19, -0.42, 0.36, 0.19, -0.62,
    -0.28, -0.14, -0.08, -0.32, -0.19, -0.42,
    -0.32, -0.19, -0.42, -0.36, -0.19, -0.62,
    -0.28, 0.14, -0.08, -0.32, 0.19, -0.42,
    -0.32, 0.19, -0.42, -0.36, 0.19, -0.62,
    0.38, -0.09, 0.08, 0.58, -0.09, 0.08,
    0.58, -0.09, 0.08, 0.58, 0.09, 0.08,
    0.58, 0.09, 0.08, 0.38, 0.09, 0.08,
  ]);
  const dog = renderer
    ? createStaticObject(dogVertices, [0.20, 0.96, 0.79, 1]) : null;

  function bodyWorldMatrix() {
    const pose = scene.poses.get('body_pose') || scene.poses.get('fastlio_odom');
    if (!pose) return null;
    const frame = resolveFrame(pose.frame_id) || identity();
    return multiply(frame, pose.matrix);
  }

  function drawObject(object, viewProjection, modelOverride = null, colorOverride = null) {
    if (!object || object.count <= 0) return;
    let model = modelOverride;
    if (!model) {
      const frame = resolveFrame(object.frame);
      if (!frame) return;
      model = object.pose ? multiply(frame, object.pose) : frame;
    }
    gl.bindBuffer(gl.ARRAY_BUFFER, object.buffer);
    gl.enableVertexAttribArray(renderer.position);
    gl.vertexAttribPointer(renderer.position, 3, gl.FLOAT, false, 0, 0);
    gl.uniformMatrix4fv(renderer.viewProjection, false, viewProjection);
    gl.uniformMatrix4fv(renderer.model, false, model);
    const color = colorOverride || object.color || [1, 1, 1, 1];
    gl.uniform4f(renderer.color, color[0], color[1], color[2], color[3]);
    gl.uniform2f(renderer.heightRange, object.heightRange[0], object.heightRange[1]);
    gl.uniform1f(renderer.colorMode, object.colorMode ? 1 : 0);
    gl.uniform1f(renderer.pointSize, object.pointSize || 2);
    const isPoints = object.primitive === 'points';
    gl.uniform1f(renderer.roundPoint, isPoints ? 1 : 0);
    let primitive = gl.LINES;
    if (isPoints) primitive = gl.POINTS;
    else if (object.primitive === 'line_strip') primitive = gl.LINE_STRIP;
    gl.drawArrays(primitive, 0, object.count);
  }

  function dynamicLineObject(values, color) {
    return createStaticObject(new Float32Array(values), color, 'lines');
  }

  function drawTrail(viewProjection) {
    if (!scene.layers.robot || scene.trail.length < 2) return;
    const points = new Float32Array(scene.trail.length * 3);
    scene.trail.forEach((point, index) => points.set(point, index * 3));
    const trailObject = createStaticObject(points, [1.0, 0.70, 0.25, 0.72], 'line_strip');
    drawObject(trailObject, viewProjection, identity());
    gl.deleteBuffer(trailObject.buffer);
  }

  function drawTf(viewProjection) {
    if (!scene.layers.tf || scene.transforms.size === 0) return;
    const links = [];
    const axes = [];
    let rendered = 0;
    for (const [child, edge] of scene.transforms) {
      if (rendered >= 100) break;
      const childMatrix = resolveFrame(child);
      const parentMatrix = resolveFrame(edge.parent);
      if (!childMatrix || !parentMatrix) continue;
      const childPoint = transformPoint(childMatrix, [0, 0, 0]);
      const parentPoint = transformPoint(parentMatrix, [0, 0, 0]);
      links.push(...parentPoint, ...childPoint);
      const scale = 0.28;
      axes.push(...childPoint, ...transformPoint(childMatrix, [scale, 0, 0]));
      axes.push(...childPoint, ...transformPoint(childMatrix, [0, scale, 0]));
      axes.push(...childPoint, ...transformPoint(childMatrix, [0, 0, scale]));
      rendered += 1;
    }
    const linksObject = dynamicLineObject(links, [0.54, 0.64, 0.72, 0.36]);
    const axesObject = dynamicLineObject(axes, [0.95, 0.75, 0.30, 0.9]);
    drawObject(linksObject, viewProjection, identity());
    drawObject(axesObject, viewProjection, identity());
    gl.deleteBuffer(linksObject.buffer);
    gl.deleteBuffer(axesObject.buffer);
  }

  function cameraGeometry() {
    const camera = scene.camera;
    const horizontal = camera.distance * Math.cos(camera.pitch);
    const eye = [
      camera.target[0] + horizontal * Math.cos(camera.yaw),
      camera.target[1] + horizontal * Math.sin(camera.yaw),
      camera.target[2] + camera.distance * Math.sin(camera.pitch),
    ];
    const forward = normalize([
      camera.target[0] - eye[0],
      camera.target[1] - eye[1],
      camera.target[2] - eye[2],
    ]);
    const right = normalize(cross(forward, [0, 0, 1]));
    const up = normalize(cross(right, forward));
    return { eye, forward, right, up };
  }

  function cameraMatrices() {
    const camera = scene.camera;
    const geometry = cameraGeometry();
    const projection = perspective(
      Math.PI / 3,
      Math.max(0.1, canvas.width / Math.max(1, canvas.height)),
      0.03,
      500,
    );
    return multiply(projection, lookAt(geometry.eye, camera.target, [0, 0, 1]));
  }

  function projectedPoint(matrix, point, rect) {
    const x = point[0]; const y = point[1]; const z = point[2];
    const clipX = matrix[0] * x + matrix[4] * y + matrix[8] * z + matrix[12];
    const clipY = matrix[1] * x + matrix[5] * y + matrix[9] * z + matrix[13];
    const clipZ = matrix[2] * x + matrix[6] * y + matrix[10] * z + matrix[14];
    const clipW = matrix[3] * x + matrix[7] * y + matrix[11] * z + matrix[15];
    if (!Number.isFinite(clipW) || clipW <= 0.0001) return null;
    const ndcX = clipX / clipW;
    const ndcY = clipY / clipW;
    const ndcZ = clipZ / clipW;
    if (ndcZ < -1 || ndcZ > 1 || Math.abs(ndcX) > 1.2 || Math.abs(ndcY) > 1.2) return null;
    return {
      x: rect.left + (ndcX + 1) * rect.width * 0.5,
      y: rect.top + (1 - ndcY) * rect.height * 0.5,
      depth: (ndcZ + 1) * 0.5,
    };
  }

  function pickCloudPoint(clientX, clientY, allowedLayers = ['registered', 'global_map']) {
    const rect = canvas.getBoundingClientRect();
    if (rect.width < 2 || rect.height < 2) return null;
    const viewProjection = cameraMatrices();
    const radius = Math.max(24, Math.min(42, Math.min(rect.width, rect.height) * 0.055));
    const radiusSquared = radius * radius;
    let best = null;

    for (const object of scene.objects.values()) {
      if (!scene.layers[object.layer] || object.primitive !== 'points'
        || !allowedLayers.includes(object.layer)) continue;
      const frame = resolveFrame(object.frame);
      if (!frame) continue;
      const model = object.pose ? multiply(frame, object.pose) : frame;
      const matrix = multiply(viewProjection, model);
      const positions = object.positions;
      // Point-cloud transfer is already bounded, but keep touch picking fast
      // if a future source raises that limit.
      const step = Math.max(3, Math.ceil(positions.length / (60000 * 3)) * 3);
      for (let index = 0; index < positions.length; index += step) {
        const local = [positions[index], positions[index + 1], positions[index + 2]];
        const projected = projectedPoint(matrix, local, rect);
        if (!projected) continue;
        const dx = projected.x - clientX;
        const dy = projected.y - clientY;
        const distanceSquared = dx * dx + dy * dy;
        if (distanceSquared > radiusSquared) continue;
        // Prefer the touched pixel, then the front-most surface when several
        // 3-D points project into the same finger-sized area.
        const score = distanceSquared + projected.depth * 3;
        if (best && score >= best.score) continue;
        best = {
          score,
          layer: object.layer,
          world: transformPoint(model, local),
          pixelDistance: Math.sqrt(distanceSquared),
        };
      }
    }
    return best;
  }

  function groundPointAt(clientX, clientY, z = 0) {
    const rect = canvas.getBoundingClientRect();
    if (rect.width < 2 || rect.height < 2) return null;
    const ndcX = ((clientX - rect.left) / rect.width) * 2 - 1;
    const ndcY = 1 - ((clientY - rect.top) / rect.height) * 2;
    const geometry = cameraGeometry();
    const spread = Math.tan(Math.PI / 6);
    const aspect = rect.width / rect.height;
    const direction = normalize([
      geometry.forward[0] + geometry.right[0] * ndcX * aspect * spread
        + geometry.up[0] * ndcY * spread,
      geometry.forward[1] + geometry.right[1] * ndcX * aspect * spread
        + geometry.up[1] * ndcY * spread,
      geometry.forward[2] + geometry.right[2] * ndcX * aspect * spread
        + geometry.up[2] * ndcY * spread,
    ]);
    if (Math.abs(direction[2]) < 1e-4) return null;
    const distance = (z - geometry.eye[2]) / direction[2];
    if (!Number.isFinite(distance) || distance <= 0) return null;
    return [
      geometry.eye[0] + direction[0] * distance,
      geometry.eye[1] + direction[1] * distance,
      z,
    ];
  }

  function resize() {
    if (!gl) return;
    const rect = canvas.getBoundingClientRect();
    if (rect.width < 2 || rect.height < 2) return;
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    const width = Math.round(rect.width * dpr);
    const height = Math.round(rect.height * dpr);
    if (canvas.width !== width || canvas.height !== height) {
      canvas.width = width;
      canvas.height = height;
      gl.viewport(0, 0, width, height);
    }
  }

  function updateFollowTarget() {
    if (!scene.camera.follow) return;
    const body = bodyWorldMatrix();
    if (!body) return;
    const point = transformPoint(body, [0, 0, 0]);
    scene.camera.target[0] += (point[0] - scene.camera.target[0]) * 0.18;
    scene.camera.target[1] += (point[1] - scene.camera.target[1]) * 0.18;
    scene.camera.target[2] += (point[2] + 0.25 - scene.camera.target[2]) * 0.18;
  }

  function draw() {
    requestAnimationFrame(draw);
    if (!scene.enabled || !gl || !renderer) return;
    resize();
    updateFollowTarget();
    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
    gl.useProgram(renderer.program);
    const viewProjection = cameraMatrices();
    gl.depthMask(true);
    drawObject(grid, viewProjection, identity());
    drawObject(axis, viewProjection, identity());

    let totalPoints = 0;
    for (const object of scene.objects.values()) {
      if (!scene.layers[object.layer]) continue;
      if (object.primitive === 'points') totalPoints += object.count;
      drawObject(object, viewProjection);
    }
    drawTrail(viewProjection);
    if (scene.layers.robot) {
      const body = bodyWorldMatrix();
      if (body) drawObject(dog, viewProjection, body);
    }
    drawTf(viewProjection);

    const now = performance.now();
    scene.frameCount += 1;
    if (now - scene.fpsTime >= 1000) {
      scene.fps = scene.frameCount * 1000 / (now - scene.fpsTime);
      scene.frameCount = 0;
      scene.fpsTime = now;
    }
    if (now - scene.readoutTime > 250) {
      scene.readoutTime = now;
      if (byId('sceneFps')) byId('sceneFps').textContent = `${scene.fps.toFixed(0)} FPS`;
      if (byId('scenePoints')) byId('scenePoints').textContent = `${totalPoints.toLocaleString()} 点`;
      if (byId('sceneCamera')) byId('sceneCamera').textContent = `相机 ${scene.camera.distance.toFixed(1)} m`;
    }
  }

  function fitScene() {
    let min = [Infinity, Infinity, Infinity];
    let max = [-Infinity, -Infinity, -Infinity];
    for (const object of scene.objects.values()) {
      if (!scene.layers[object.layer] || object.positions.length === 0) continue;
      const frame = resolveFrame(object.frame);
      if (!frame) continue;
      const model = object.pose ? multiply(frame, object.pose) : frame;
      const step = Math.max(3, Math.floor(object.positions.length / 15000 / 3) * 3);
      for (let index = 0; index < object.positions.length; index += step) {
        const point = transformPoint(model, [
          object.positions[index],
          object.positions[index + 1],
          object.positions[index + 2],
        ]);
        for (let axisIndex = 0; axisIndex < 3; axisIndex += 1) {
          min[axisIndex] = Math.min(min[axisIndex], point[axisIndex]);
          max[axisIndex] = Math.max(max[axisIndex], point[axisIndex]);
        }
      }
    }
    if (!Number.isFinite(min[0])) {
      const body = bodyWorldMatrix();
      if (body) {
        const point = transformPoint(body, [0, 0, 0]);
        scene.camera.target = [point[0], point[1], point[2]];
        scene.camera.distance = 8;
      }
      return;
    }
    scene.camera.target = [
      (min[0] + max[0]) / 2,
      (min[1] + max[1]) / 2,
      (min[2] + max[2]) / 2,
    ];
    scene.camera.distance = Math.max(
      3,
      Math.min(120, Math.hypot(max[0] - min[0], max[1] - min[1], max[2] - min[2]) * 0.78),
    );
    scene.camera.follow = false;
    byId('sceneFollow')?.classList.remove('active');
  }

  function resetVisualization() {
    // The global PCD is the selectable reference map rather than transient
    // history. Keep it (plus current poses/TF), and clear every client-side
    // overlay that can become stale. Live ROS topics repopulate themselves.
    [...scene.objects.keys()].forEach((key) => {
      if (!['cloud:global_map', 'cloud:traversable'].includes(key)) removeObject(key);
    });
    scene.trail = [];
    scene.pendingGoal = null;
    scene.routeWaypoints = [];
    scene.pointers.clear();
    scene.gesture = null;
    scene.primaryGesture = null;
    scene.camera = defaultCamera();
    updateGoalEditor(false);
    updateRouteMarker();
    setInteractionMode('orbit');
    byId('sceneFollow')?.classList.remove('active');

    if (scene.objects.has('cloud:global_map') || scene.objects.has('cloud:traversable')) {
      scene.autoFitDone = true;
      scene.dataReady = true;
      byId('sceneEmpty')?.classList.add('hidden');
      requestAnimationFrame(fitScene);
    } else {
      scene.autoFitDone = false;
      scene.dataReady = false;
      byId('sceneEmpty')?.classList.remove('hidden');
    }
    if (typeof window.showToast === 'function') {
      window.showToast('可视化已重置；定位和规划仍在运行');
    }
  }

  function setMode(use3d) {
    scene.enabled = Boolean(use3d);
    document.body.classList.toggle('scene-mode', scene.enabled);
    byId('show3dView')?.classList.toggle('active', scene.enabled);
    byId('show2dView')?.classList.toggle('active', !scene.enabled);
    if (scene.enabled) {
      requestAnimationFrame(resize);
      if (typeof window.nav2Send === 'function') {
        window.nav2Send({ type: 'request_scene_snapshot' });
      }
    } else if (typeof window.resizeNav2Canvas === 'function') {
      requestAnimationFrame(window.resizeNav2Canvas);
    }
  }

  function pointerPosition(event) {
    return { x: event.clientX, y: event.clientY };
  }

  function stopFollowing() {
    scene.camera.follow = false;
    byId('sceneFollow')?.classList.remove('active');
  }

  function setInteractionMode(mode) {
    scene.interactionMode = mode;
    byId('scenePan')?.classList.toggle('active', mode === 'pan');
    byId('sceneOrbit')?.classList.toggle('active', mode === 'orbit');
    byId('sceneGoal')?.classList.toggle('active', mode === 'goal');
    byId('sceneRoute')?.classList.toggle('active', mode === 'route');
    byId('sceneInitialPose')?.classList.toggle('active', mode === 'initial_pose');
    const hints = {
      pan: '单指平移地图 · 双指缩放和平移',
      orbit: '单指 360° 环绕 · 双指缩放和平移',
      goal: scene.pctMode
        ? '触摸绿色 PCT 可通行区域 · 拖动设置朝向'
        : '触摸当前 FAST-LIO 局部点云 · 拖动设置朝向',
      route: '依次触摸绿色 PCT 可通行区域 · 每个点自动吸附到可规划网格',
      initial_pose: '在 PCT 地图上按住位置并拖动，给出重定位的大致位置和朝向',
    };
    if (byId('sceneGestureHint')) byId('sceneGestureHint').textContent = hints[mode];
    canvas.style.cursor = ['goal', 'route', 'initial_pose'].includes(mode) ? 'crosshair' : 'grab';
    renderRouteEditor();
  }

  function selectableLayers(mode) {
    if (mode === 'route' || (mode === 'goal' && scene.pctMode)) {
      return ['traversable'];
    }
    if (mode === 'initial_pose') {
      return ['traversable'];
    }
    return ['registered'];
  }

  function bodyYaw() {
    const body = bodyWorldMatrix();
    return body ? Math.atan2(body[1], body[0]) : 0;
  }

  function goalVertices(goal) {
    const values = [];
    const radius = Math.max(0.22, Math.min(0.9, scene.camera.distance * 0.025));
    const z = (Number(goal.z) || 0) + 0.055;
    const segments = 28;
    for (let index = 0; index < segments; index += 1) {
      const first = index * Math.PI * 2 / segments;
      const second = (index + 1) * Math.PI * 2 / segments;
      values.push(
        goal.x + Math.cos(first) * radius,
        goal.y + Math.sin(first) * radius,
        z,
        goal.x + Math.cos(second) * radius,
        goal.y + Math.sin(second) * radius,
        z,
      );
    }
    const length = radius * 2.8;
    const tip = [
      goal.x + Math.cos(goal.yaw) * length,
      goal.y + Math.sin(goal.yaw) * length,
    ];
    values.push(goal.x, goal.y, z, tip[0], tip[1], z);
    const groundZ = Number.isFinite(Number(goal.groundZ))
      ? Number(goal.groundZ) : Number(goal.z) - scene.bodyHeight;
    values.push(goal.x, goal.y, groundZ + 0.03, goal.x, goal.y, z);
    const head = radius * 0.75;
    values.push(
      tip[0], tip[1], z,
      tip[0] + Math.cos(goal.yaw + 2.55) * head,
      tip[1] + Math.sin(goal.yaw + 2.55) * head,
      z,
      tip[0], tip[1], z,
      tip[0] + Math.cos(goal.yaw - 2.55) * head,
      tip[1] + Math.sin(goal.yaw - 2.55) * head,
      z,
    );
    return new Float32Array(values);
  }

  function updateGoalMarker(goal, published = false, key = 'ui:scanplanner-goal') {
    uploadObject(key, {
      layer: 'planning',
      frame: scene.fixedFrame,
      positions: goalVertices(goal),
      primitive: 'lines',
      color: key === 'ui:initial-pose'
        ? (published ? [0.22, 0.82, 1.0, 1] : [1.0, 0.72, 0.18, 1])
        : (published ? [0.22, 0.96, 0.66, 1] : [1.0, 0.72, 0.18, 1]),
      colorMode: false,
      pointSize: 3,
      heightRange: [0, 1],
    });
  }

  function updateGoalEditor(visible) {
    const goal = scene.pendingGoal;
    if (goal && byId('sceneGoalCoordinates')) {
      byId('sceneGoalCoordinates').textContent = `x ${goal.x.toFixed(2)} · y ${goal.y.toFixed(2)} · yaw ${(goal.yaw * 180 / Math.PI).toFixed(0)}°`;
    }
    const routeMode = scene.interactionMode === 'route';
    const initialPoseMode = scene.interactionMode === 'initial_pose';
    if (goal && byId('sceneGoalHeight')) {
      byId('sceneGoalHeight').textContent = `${goal.sourceLayer === 'traversable' ? 'PCT 可通行地面' : '点云地面'} Z ${goal.groundZ.toFixed(2)} m`;
    }
    if (byId('sceneGoalEditorTitle')) {
      byId('sceneGoalEditorTitle').textContent = initialPoseMode
        ? 'RELOCALIZATION INITIAL POSE · MAP'
        : routeMode ? 'PCT WAYPOINT · TRAVERSABLE MAP'
          : 'SCAN-PLANNER / PCT GOAL · MAP';
    }
    if (byId('sceneGoalEditorHint')) {
      byId('sceneGoalEditorHint').textContent = initialPoseMode
        ? '只需大致位置和朝向；发送后 ICP 会用实时雷达点云精确对齐'
        : routeMode ? '加入后按列表顺序经过；每个点已吸附到 PCT 可通行网格'
          : scene.pctMode ? '确认后交给 PCT 生成全局轨迹'
            : '确认后交给 SCAN-Planner 模式 1 进行局部规划';
    }
    byId('sceneGoalAdd')?.classList.toggle('hidden', !routeMode);
    byId('sceneGoalSend')?.classList.toggle('hidden', routeMode);
    if (byId('sceneGoalSend')) {
      byId('sceneGoalSend').textContent = initialPoseMode ? '发送粗定位' : '发送单目标';
    }
    byId('sceneGoalEditor')?.classList.toggle('visible', Boolean(visible && goal));
  }

  function routeVertices() {
    const values = [];
    scene.routeWaypoints.forEach((waypoint, index) => {
      values.push(...goalVertices(waypoint));
      if (index > 0) {
        const previous = scene.routeWaypoints[index - 1];
        values.push(
          previous.x, previous.y, previous.z + 0.07,
          waypoint.x, waypoint.y, waypoint.z + 0.07,
        );
      }
    });
    return new Float32Array(values);
  }

  function updateRouteMarker() {
    if (!scene.routeWaypoints.length) {
      removeObject('ui:pct-waypoint-route');
      return;
    }
    uploadObject('ui:pct-waypoint-route', {
      layer: 'planning',
      frame: scene.fixedFrame,
      positions: routeVertices(),
      primitive: 'lines',
      color: [0.22, 0.96, 0.66, 1],
      colorMode: false,
      pointSize: 3,
      heightRange: [0, 1],
    });
  }

  function renderRouteEditor() {
    const panel = byId('sceneRouteEditor');
    const list = byId('sceneRouteList');
    if (!panel || !list) return;
    panel.classList.toggle(
      'visible', scene.pctMode
        && (scene.interactionMode === 'route' || scene.routeWaypoints.length > 0));
    byId('sceneRouteCount').textContent = `${scene.routeWaypoints.length} 点`;
    list.replaceChildren();
    if (!scene.routeWaypoints.length) {
      const empty = document.createElement('span');
      empty.textContent = '切换“多点路线”，依次在地图上添加途经点';
      list.appendChild(empty);
    } else {
      scene.routeWaypoints.forEach((waypoint, index) => {
        const row = document.createElement('div');
        const number = document.createElement('b');
        number.textContent = `#${index + 1}`;
        const coordinates = document.createElement('code');
        coordinates.textContent = `x ${waypoint.x.toFixed(2)} · y ${waypoint.y.toFixed(2)} · 地面 ${waypoint.groundZ.toFixed(2)} · 机身 ${waypoint.z.toFixed(2)}`;
        const remove = document.createElement('button');
        remove.type = 'button';
        remove.textContent = '删除';
        remove.addEventListener('click', () => {
          scene.routeWaypoints.splice(index, 1);
          updateRouteMarker();
          renderRouteEditor();
        });
        row.append(number, coordinates, remove);
        list.appendChild(row);
      });
    }
    byId('sceneRouteUndo').disabled = !scene.routeWaypoints.length;
    byId('sceneRouteClear').disabled = !scene.routeWaypoints.length;
    byId('sceneRouteSend').disabled = !scene.routeWaypoints.length;
    if (byId('sceneRouteHeight')) {
      byId('sceneRouteHeight').textContent = '仅选择 PCT 可通行网格';
    }
  }

  function cancelPendingGoal() {
    scene.pendingGoal = null;
    removeObject('ui:scanplanner-goal');
    if (scene.interactionMode === 'initial_pose') removeObject('ui:initial-pose');
    updateGoalEditor(false);
    setInteractionMode('orbit');
  }

  function gestureMetrics() {
    const points = [...scene.pointers.values()];
    if (points.length < 2) return null;
    return {
      distance: Math.hypot(points[0].x - points[1].x, points[0].y - points[1].y),
      midpoint: {
        x: (points[0].x + points[1].x) / 2,
        y: (points[0].y + points[1].y) / 2,
      },
    };
  }

  canvas.addEventListener('pointerdown', (event) => {
    canvas.setPointerCapture(event.pointerId);
    const position = pointerPosition(event);
    scene.pointers.set(event.pointerId, position);
    if (scene.pointers.size === 1) {
      const editingPoint = ['goal', 'route', 'initial_pose'].includes(scene.interactionMode);
      const picked = editingPoint
        ? pickCloudPoint(position.x, position.y, selectableLayers(scene.interactionMode)) : null;
      const anchor = editingPoint
        ? picked?.world || null : groundPointAt(position.x, position.y, 0);
      scene.primaryGesture = {
        pointerId: event.pointerId,
        start: position,
        anchor,
        moved: false,
      };
      if (editingPoint && anchor) {
        const groundZ = anchor[2];
        scene.pendingGoal = {
          x: anchor[0],
          y: anchor[1],
          groundZ,
          bodyHeight: scene.bodyHeight,
          z: groundZ + scene.bodyHeight,
          yaw: bodyYaw(),
          sourceLayer: picked?.layer || '',
        };
        updateGoalMarker(
          scene.pendingGoal,
          false,
          scene.interactionMode === 'initial_pose' ? 'ui:initial-pose' : 'ui:scanplanner-goal',
        );
        updateGoalEditor(false);
      } else if (editingPoint) {
        scene.pendingGoal = null;
        removeObject('ui:scanplanner-goal');
        updateGoalEditor(false);
        if (byId('sceneGestureHint')) {
          byId('sceneGestureHint').textContent = ['route', 'initial_pose'].includes(scene.interactionMode)
            ? '没有选中绿色可通行网格；请等待 PCT 图层就绪并触摸绿色区域'
            : '没有选中局部点云；请放大后触摸可见地面';
        }
      }
    } else {
      scene.primaryGesture = null;
      scene.gesture = gestureMetrics();
    }
  });

  canvas.addEventListener('pointermove', (event) => {
    const previous = scene.pointers.get(event.pointerId);
    if (!previous) return;
    const current = pointerPosition(event);
    scene.pointers.set(event.pointerId, current);
    if (scene.pointers.size === 1) {
      const gesture = scene.primaryGesture;
      if (gesture) {
        gesture.moved = gesture.moved || Math.hypot(
          current.x - gesture.start.x,
          current.y - gesture.start.y,
        ) > 6;
      }
      if (['goal', 'route', 'initial_pose'].includes(scene.interactionMode)) {
        const point = groundPointAt(
          current.x, current.y, scene.pendingGoal?.groundZ ?? 0);
        if (point && scene.pendingGoal && gesture?.anchor) {
          const dx = point[0] - gesture.anchor[0];
          const dy = point[1] - gesture.anchor[1];
          if (gesture.moved && Math.hypot(dx, dy) > 0.03) {
            scene.pendingGoal.yaw = Math.atan2(dy, dx);
          }
          updateGoalMarker(
            scene.pendingGoal,
            false,
            scene.interactionMode === 'initial_pose' ? 'ui:initial-pose' : 'ui:scanplanner-goal',
          );
          updateGoalEditor(false);
        }
        return;
      }
      stopFollowing();
      if (scene.interactionMode === 'orbit') {
        scene.camera.yaw -= (current.x - previous.x) * 0.006;
        scene.camera.yaw = Math.atan2(
          Math.sin(scene.camera.yaw),
          Math.cos(scene.camera.yaw),
        );
        scene.camera.pitch = Math.max(
          -1.45,
          Math.min(1.45, scene.camera.pitch + (current.y - previous.y) * 0.005),
        );
      } else {
        const before = groundPointAt(previous.x, previous.y);
        const after = groundPointAt(current.x, current.y);
        if (before && after) {
          scene.camera.target[0] += before[0] - after[0];
          scene.camera.target[1] += before[1] - after[1];
        }
      }
      return;
    }
    const metrics = gestureMetrics();
    if (!metrics || !scene.gesture) {
      scene.gesture = metrics;
      return;
    }
    const before = groundPointAt(
      scene.gesture.midpoint.x,
      scene.gesture.midpoint.y,
    );
    const ratio = scene.gesture.distance / Math.max(1, metrics.distance);
    scene.camera.distance = Math.max(0.8, Math.min(180, scene.camera.distance * ratio));
    const after = groundPointAt(metrics.midpoint.x, metrics.midpoint.y);
    if (before && after) {
      scene.camera.target[0] += before[0] - after[0];
      scene.camera.target[1] += before[1] - after[1];
    }
    stopFollowing();
    scene.gesture = metrics;
  });

  function releasePointer(event, cancelled = false) {
    const wasOnlyPointer = scene.pointers.size === 1;
    if (!cancelled && wasOnlyPointer && ['goal', 'route', 'initial_pose'].includes(scene.interactionMode)
      && scene.primaryGesture?.pointerId === event.pointerId && scene.pendingGoal) {
      updateGoalMarker(
        scene.pendingGoal,
        false,
        scene.interactionMode === 'initial_pose' ? 'ui:initial-pose' : 'ui:scanplanner-goal',
      );
      updateGoalEditor(true);
    }
    scene.pointers.delete(event.pointerId);
    try { canvas.releasePointerCapture(event.pointerId); } catch (_error) { /* no-op */ }
    scene.gesture = scene.pointers.size > 1 ? gestureMetrics() : null;
    const remaining = [...scene.pointers.entries()][0];
    scene.primaryGesture = remaining ? {
      pointerId: remaining[0],
      start: remaining[1],
      anchor: groundPointAt(remaining[1].x, remaining[1].y),
      moved: false,
    } : null;
  }
  canvas.addEventListener('pointerup', releasePointer);
  canvas.addEventListener('pointercancel', (event) => releasePointer(event, true));
  canvas.addEventListener('wheel', (event) => {
    event.preventDefault();
    const before = groundPointAt(event.clientX, event.clientY);
    scene.camera.distance = Math.max(
      0.8,
      Math.min(180, scene.camera.distance * Math.exp(event.deltaY * 0.0012)),
    );
    const after = groundPointAt(event.clientX, event.clientY);
    if (before && after) {
      scene.camera.target[0] += before[0] - after[0];
      scene.camera.target[1] += before[1] - after[1];
    }
    stopFollowing();
  }, { passive: false });
  canvas.addEventListener('contextmenu', (event) => event.preventDefault());

  document.querySelectorAll('[data-scene-layer]').forEach((input) => {
    input.addEventListener('change', () => {
      scene.layers[input.dataset.sceneLayer] = input.checked;
    });
  });
  byId('show3dView')?.addEventListener('click', () => setMode(true));
  byId('show2dView')?.addEventListener('click', () => setMode(false));
  byId('scenePan')?.addEventListener('click', () => {
    stopFollowing();
    setInteractionMode('pan');
  });
  byId('sceneOrbit')?.addEventListener('click', () => {
    stopFollowing();
    setInteractionMode('orbit');
  });
  byId('sceneTop')?.addEventListener('click', () => {
    stopFollowing();
    scene.camera.pitch = 1.515;
    scene.camera.yaw = -Math.PI / 2;
    setInteractionMode('pan');
  });
  byId('sceneFollow')?.addEventListener('click', () => {
    scene.camera.follow = !scene.camera.follow;
    byId('sceneFollow')?.classList.toggle('active', scene.camera.follow);
    if (scene.camera.follow) setInteractionMode('pan');
  });
  byId('sceneFit')?.addEventListener('click', fitScene);
  byId('sceneReset')?.addEventListener('click', resetVisualization);
  byId('sceneGoal')?.addEventListener('click', () => {
    stopFollowing();
    setInteractionMode(scene.interactionMode === 'goal' ? 'orbit' : 'goal');
  });
  byId('sceneRoute')?.addEventListener('click', () => {
    stopFollowing();
    setInteractionMode(scene.interactionMode === 'route' ? 'orbit' : 'route');
  });
  byId('sceneInitialPose')?.addEventListener('click', () => {
    stopFollowing();
    setInteractionMode(scene.interactionMode === 'initial_pose' ? 'orbit' : 'initial_pose');
  });
  byId('sceneGoalCancel')?.addEventListener('click', cancelPendingGoal);
  byId('sceneGoalAdd')?.addEventListener('click', () => {
    if (!scene.pendingGoal || scene.interactionMode !== 'route') return;
    scene.routeWaypoints.push({ ...scene.pendingGoal });
    scene.pendingGoal = null;
    removeObject('ui:scanplanner-goal');
    updateGoalEditor(false);
    updateRouteMarker();
    renderRouteEditor();
  });
  byId('sceneGoalSend')?.addEventListener('click', () => {
    const goal = scene.pendingGoal;
    if (!goal || typeof window.nav2Send !== 'function') return;
    const initialPose = scene.interactionMode === 'initial_pose';
    if (window.nav2Send({
      type: initialPose ? 'initial_pose' : 'scanplanner_goal',
      x: goal.x,
      y: goal.y,
      z: initialPose ? goal.groundZ : goal.z,
      yaw: goal.yaw,
    })) {
      updateGoalEditor(false);
    }
  });
  byId('sceneFullscreen')?.addEventListener('click', () => {
    const enabled = !document.body.classList.contains('scene-fullscreen');
    document.body.classList.toggle('scene-fullscreen', enabled);
    byId('sceneFullscreen').textContent = enabled ? '退出全屏' : '全屏';
    requestAnimationFrame(resize);
  });
  byId('sceneRouteUndo')?.addEventListener('click', () => {
    scene.routeWaypoints.pop();
    updateRouteMarker();
    renderRouteEditor();
  });
  byId('sceneRouteClear')?.addEventListener('click', () => {
    scene.routeWaypoints = [];
    updateRouteMarker();
    renderRouteEditor();
  });
  byId('sceneRouteSend')?.addEventListener('click', () => {
    if (!scene.routeWaypoints.length || typeof window.nav2Send !== 'function') return;
    window.nav2Send({
      type: 'pct_waypoints',
      waypoints: scene.routeWaypoints.map((waypoint) => ({ ...waypoint })),
    });
  });
  byId('sceneLayerCollapse')?.addEventListener('click', () => {
    const panel = byId('sceneLayers');
    const collapsed = panel.classList.toggle('collapsed');
    byId('sceneLayerCollapse').textContent = collapsed ? '+' : '−';
  });

  window.addEventListener('resize', resize);
  window.addEventListener('orientationchange', () => setTimeout(resize, 120));
  document.body.classList.add('scene-mode');
  setInteractionMode('orbit');
  renderRouteEditor();
  if (!gl || !renderer) {
    const empty = byId('sceneEmpty');
    if (empty) {
      empty.querySelector('strong').textContent = '此设备不支持 WebGL';
      empty.querySelector('span').textContent = '请在系统设置中启用 WebView 硬件加速';
    }
  }

  window.scanScene = {
    handleMessage,
    setMode,
    resize,
    fitScene,
    resetVisualization,
    setBodyHeight(value) {
      const height = Number(value);
      if (!Number.isFinite(height) || height <= 0 || height > 2) return;
      scene.bodyHeight = height;
      renderRouteEditor();
    },
    setWorkflow(active) {
      const profileId = active?.running ? String(active.profile_id || '') : '';
      if (profileId === scene.activeProfile) return;
      scene.activeProfile = profileId;
      scene.pctMode = profileId === 'pct_offline_demo'
        || profileId.startsWith('pct_scanplanner');
      const defaults = scene.pctMode
        ? { traversable: true, registered: false, occupancy: false, inflated: false, global_map: false }
        : { traversable: false, registered: true, occupancy: true, inflated: false, global_map: false };
      Object.entries(defaults).forEach(([layer, visible]) => {
        scene.layers[layer] = visible;
        const input = document.querySelector(`[data-scene-layer="${layer}"]`);
        if (input) input.checked = visible;
      });
      byId('sceneRoute')?.classList.toggle('hidden', !scene.pctMode);
      byId('sceneInitialPose')?.classList.toggle('hidden', !scene.pctMode);
      if (!scene.pctMode) {
        scene.pendingGoal = null;
        scene.routeWaypoints = [];
        removeObject('ui:scanplanner-goal');
        removeObject('ui:pct-waypoint-route');
        removeObject('ui:initial-pose');
        removeObject('path:/initial_path');
        updateGoalEditor(false);
      }
      setInteractionMode('orbit');
    },
    goalPublished(goal) {
      if (goal && Number.isFinite(Number(goal.x)) && Number.isFinite(Number(goal.y))) {
        scene.pendingGoal = {
          x: Number(goal.x),
          y: Number(goal.y),
          z: Number(goal.z) || 0,
          groundZ: (Number(goal.z) || 0) - scene.bodyHeight,
          bodyHeight: scene.bodyHeight,
          yaw: Number(goal.yaw) || 0,
        };
        updateGoalMarker(scene.pendingGoal, true);
      }
      updateGoalEditor(false);
      setInteractionMode('orbit');
    },
    routePublished(waypoints) {
      if (!scene.pctMode) return;
      if (Array.isArray(waypoints)) {
        scene.routeWaypoints = waypoints.map((waypoint) => ({
          x: Number(waypoint.x) || 0,
          y: Number(waypoint.y) || 0,
          z: Number(waypoint.z) || 0,
          groundZ: (Number(waypoint.z) || 0) - scene.bodyHeight,
          bodyHeight: scene.bodyHeight,
          yaw: Number(waypoint.yaw) || 0,
        }));
        updateRouteMarker();
        renderRouteEditor();
      }
      setInteractionMode('orbit');
    },
    initialPosePublished(pose) {
      if (pose && Number.isFinite(Number(pose.x)) && Number.isFinite(Number(pose.y))) {
        const groundZ = Number(pose.z) || 0;
        const marker = {
          x: Number(pose.x),
          y: Number(pose.y),
          groundZ,
          bodyHeight: scene.bodyHeight,
          z: groundZ + scene.bodyHeight,
          yaw: Number(pose.yaw) || 0,
          sourceLayer: 'traversable',
        };
        updateGoalMarker(marker, true, 'ui:initial-pose');
      }
      scene.pendingGoal = null;
      updateGoalEditor(false);
      setInteractionMode('orbit');
    },
    connectionChanged(connected) {
      scene.connected = Boolean(connected);
      if (!connected && !scene.dataReady) byId('sceneEmpty')?.classList.remove('hidden');
    },
  };
  requestAnimationFrame(draw);
}());
