if (!Detector.webgl) Detector.addGetWebGLMessage();

var container, stats;
var camera, controls, scene, renderer;

var box;
var center;
var points;
var sphereInter;
var INTERSECTED = null;

var willRemovedPoints = [];

var kVTKModels = [
  {
    color: 0xCD5C5C,
    url: 'models/1001/38-2.vtk'
  },
  {
    color: 0xF08080,
    url: 'models/1001/52-2.vtk'
  },
  {
    color: 0xFA8072,
    url: 'models/1001/82-2.vtk'
  },
  {
    color: 0xE9967A,
    url: 'models/1001/88-2.vtk'
  },
  {
    color: 0xFFA07A,
    url: 'models/1001/164-2.vtk'
  },
  {
    color: 0xABB2B9,
    url: 'models/1001/205-2.vtk'
  },
  {
    color: 0xAAB7B8,
    url: 'models/1001/244-2.vtk'
  }
];

init();
animate();

function updateGroupGeometry(mesh, geometry) {
  mesh.children[0].geometry.dispose();
  mesh.children[1].geometry.dispose();

  mesh.children[0].geometry = new THREE.WireframeGeometry(geometry);
  mesh.children[1].geometry = geometry;

  // these do not update nicely together if shared
}

function loadOBJModel() {
  var loader = new THREE.OBJLoader();
  loader.load('models/1001/38-1.obj', function (object) {
    /*
    geometry.center();
    geometry.computeVertexNormals();
    geometry.computeFaceNormals();

    var material = new THREE.MeshLambertMaterial({
    // var material = new THREE.MeshPhongMaterial({
      color: 0xffffff,
      wireframe: false,
      // THREE.FrontSide, THREE.BackSide, THREE.DoubleSide
      side: THREE.DoubleSide
    });
    var mesh = new THREE.Mesh(geometry, material);
    mesh.doubleSided = true;
    mesh.scale.multiplyScalar(400);
    */

    var mesh = object.children[0];
    mesh.scale.multiplyScalar(400);

    var geometry = mesh.geometry;
    // geometry.center();
    // geometry.computeVertexNormals(true);
    // geometry.computeFaceNormals();

    var newGeometry = new THREE.Geometry().fromBufferGeometry(geometry);
    newGeometry.center();
    // newGeometry.mergeVertices();
    // newGeometry.computeVertexNormals(true);
    // newGeometry.computeFaceNormals();
    mesh.geometry = new THREE.BufferGeometry().fromGeometry(newGeometry);

    var material = mesh.material;
    material.color = new THREE.Color(0x789abc);
    material.side = THREE.DoubleSide;
    material.flatShading = true;

    // var mesh = new THREE.MeshPhongMaterial({
    var mesh = new THREE.MeshBasicMaterial({
        color: 0x789abc,
        side: THREE.DoubleSide,
        flatShading: true,
        shading: THREE.SmoothShading
    });

    scene.add(object);

    // var helper = new THREE.VertexNormalsHelper(mesh, 2, 0x00ff00, 1);
    // scene.add(helper);
  });
}

async function loadAllVTKModels() {
  var group = new THREE.Group();
  scene.add(group);

  var models = kVTKModels;
  for (var i = 0; i < models.length; i++) {
    var config = models[i];
    var mesh = await loadVTKModel(config);
    group.add(mesh);
  }

  /*
  var box = new THREE.BoxHelper(group, 0xffffff);
  scene.add(box);
  box.geometry.center();
  var { x, y, z } = box.geometry.boundingSphere.center;
  box.translateX(-x);
  box.translateY(-y);
  box.translateZ(-z);
  box.update();
  */
}

function loadVTKModel(config) {
  return new Promise(resolve => {
    var loader = new THREE.VTKLoader();
    var url = config.url; // .replace('-1', '');
    loader.load(url, function(geometry) {
      // var subdivisions = 1;
      // var modifier = new THREE.BufferSubdivisionModifier(subdivisions);
      // geometry = modifier.modify(geometry);

      // geometry.center();
      geometry.computeVertexNormals(true);
      geometry.computeFaceNormals();

      var material = new THREE.MeshPhongMaterial({
        color: new THREE.Color(config.color),
        side: 2,
        reflectivity: 0,
        flatShading: true
      });
      var mesh = new THREE.Mesh(geometry, material);
      mesh.scale.multiplyScalar(100);
      resolve(mesh);
    });
  });
}

function addLights() {
  var dirLight = new THREE.DirectionalLight(0xffffff);
  dirLight.position.set(200, 200, 1000).normalize();
  camera.add(dirLight);
  camera.add(dirLight.target);

  var lights = [];
  lights[0] = new THREE.PointLight(0xffffff, 1, 0);
  lights[1] = new THREE.PointLight(0xffffff, 1, 0);
  lights[2] = new THREE.PointLight(0xffffff, 1, 0);

  lights[0].position.set(0, 200, 0);
  lights[1].position.set(100, 200, 100);
  lights[2].position.set(-100, -200, -100);

  scene.add(lights[0]);
  scene.add(lights[1]);
  scene.add(lights[2]);
}

function getKeyPoints() {
  return G_DATA.vessels[0].vertexes.map(
    ({ x, y, z }) => new THREE.Vector3(x, y, z)
  );
}

function drawCurve() {
  // var curve = new THREE.CatmullRomCurve3([
  //   new THREE.Vector3(-10, 0, 10),
  //   new THREE.Vector3(-5, 5, 5),
  //   new THREE.Vector3(0, 0, 0),
  //   new THREE.Vector3(5, -5, 5),
  //   new THREE.Vector3(10, 0, 10)
  // ]);
  var curve = new THREE.CatmullRomCurve3(getKeyPoints());
  // var points = curve.getPoints(50);
  // var geometry = new THREE.BufferGeometry().setFromPoints(points);
  // var material = new THREE.LineBasicMaterial({
  //   color: 0xff0000,
  //   linewidth: 50
  // });
  // var curveObject = new THREE.Line(geometry, material);
  // scene.add(curveObject);

  var geometry = new THREE.TubeGeometry(curve, 20, 1, 8, false);

  var subdivisions = 2;
  var modifier = new THREE.BufferSubdivisionModifier(subdivisions);
  geometry = modifier.modify(geometry);
  // var material = new THREE.MeshBasicMaterial({ color: 0xff0000 });
  // var mesh = new THREE.Mesh(geometry, material);
  // scene.add(mesh);

  var mesh = new THREE.Object3D();
  mesh.add(
    new THREE.LineSegments(
      new THREE.Geometry(),
      new THREE.LineBasicMaterial({
        color: 0xffffff,
        transparent: true,
        opacity: 0.5
      })
    )
  );

  mesh.add(
    new THREE.Mesh(
      new THREE.Geometry(),
      new THREE.MeshPhongMaterial({
        color: 0x156289,
        emissive: 0x072534,
        side: THREE.DoubleSide,
        flatShading: true
      })
    )
  );
  scene.add(mesh);

  updateGroupGeometry(mesh, geometry);

  var { x, y, z } = center;
  mesh.translateX(-x);
  mesh.translateY(-y + 0);
  mesh.translateZ(-z);
}

function drawPointCloud() {
  var positions = [];
  var colors = [];
  var originalColors = [];
  var geometry = new THREE.BufferGeometry();
  var vessels = G_DATA.vessels;
  vessels.forEach(vessel => {
    // const color = new THREE.Color(vessel.color);
    vessel.vertexes.forEach(({ x, y, z, color }) => {
      color = new THREE.Color(color || vessel.color);
      positions.push(x, y, z);
      // const color = getColor(x, y, z);
      colors.push(color.r, color.g, color.b);
      originalColors.push(color.r, color.g, color.b);
    });
  });
  geometry.addAttribute(
    'position',
    new THREE.Float32BufferAttribute(positions, 3)
  );
  geometry.addAttribute('color', new THREE.Float32BufferAttribute(colors, 3));
  geometry.addAttribute(
    'xcolor',
    new THREE.Float32BufferAttribute(originalColors, 3)
  );
  geometry.computeBoundingSphere();

  var material = new THREE.PointsMaterial({
    size: 5,
    vertexColors: THREE.VertexColors
  });
  points = new THREE.Points(geometry, material);
  scene.add(points);

  box = new THREE.BoxHelper(points, 0xffffff);
  scene.add(box);

  var { x, y, z } = box.geometry.boundingSphere.center;
  center = { x, y, z };
  // var center = new THREE.Vector3().copy(center);
  points.translateX(-x);
  points.translateY(-y);
  points.translateZ(-z);
  points.visible = false;
  box.update();
}

function addSphereInter() {
  sphereInter = new THREE.Mesh(
    new THREE.SphereGeometry(2),
    new THREE.MeshBasicMaterial({ color: 0x00d061 })
  );
  sphereInter.visible = false;
  scene.add(sphereInter);
}

function init() {
  scene = new THREE.Scene();
  scene.background = new THREE.Color(0x050505);
  scene.fog = new THREE.Fog(0x050505, 2000, 3500);

  var ratio = window.innerWidth / window.innerHeight;
  // camera = new THREE.PerspectiveCamera(27, ratio, 5, 3500);
  // camera.position.z = 500;
  camera = new THREE.PerspectiveCamera(27, ratio, 5, 3500);
  camera.position.z = 600;
  scene.add(camera);

  controls = new THREE.TrackballControls(camera);
  controls.rotateSpeed = 5.0;
  controls.zoomSpeed = 5;
  controls.panSpeed = 2;
  controls.noZoom = false;
  controls.noPan = false;
  controls.staticMoving = true;
  controls.dynamicDampingFactor = 0.3;

  drawPointCloud();
  addLights();
  drawCurve();
  addSphereInter();
  // loadVTKModel();
  loadAllVTKModels();
  // loadOBJModel();


  scene.add(new THREE.AxisHelper(1000));

  renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(window.devicePixelRatio);
  renderer.setSize(window.innerWidth, window.innerHeight);
  renderer.setClearColor(0x000000, 1);
  document.body.appendChild(renderer.domElement);

  // controls = new THREE.OrbitControls(camera, renderer.domElement);

  stats = new Stats();
  container = document.createElement('div');
  document.body.appendChild(container);
  container.appendChild(stats.dom);

  window.addEventListener('resize', onWindowResize, false);
  document.body.addEventListener('mousemove', onMouseMove, false);
}

function resetColor(attributes, index) {
  const r = attributes.xcolor.getX(index);
  const g = attributes.xcolor.getY(index);
  const b = attributes.xcolor.getZ(index);
  attributes.color.setXYZ(index, r, g, b);
  attributes.color.needsUpdate = true; // eslint-disable-line
}

function onMouseMove(e) {
  e.preventDefault();
  var mouse = new THREE.Vector2();
  var x = 0;
  var y = 0;
  var width = window.innerWidth;
  var height = window.innerHeight;
  mouse.x = (e.clientX - x) / width * window.devicePixelRatio - 1;
  mouse.y = -((e.clientY - y) / height) * window.devicePixelRatio + 1;

  var geometry = points.geometry;
  var attributes = geometry.attributes;

  var raycaster = new THREE.Raycaster();
  raycaster.linePrecision = 3;
  raycaster.setFromCamera(mouse, camera);

  var intersects = raycaster.intersectObject(points);
  if (intersects.length) {
    const index = intersects[0].index;
    if (INTERSECTED !== index) {
      // resetColor(attributes, INTERSECTED);
      INTERSECTED = index;
      const red = new THREE.Color(0xf52431);
      attributes.color.setXYZ(INTERSECTED, red.r, red.g, red.b);
      attributes.color.needsUpdate = true;
      sphereInter.visible = true;
      sphereInter.position.copy(intersects[0].point);

      var x = attributes.position.getX(INTERSECTED);
      var y = attributes.position.getY(INTERSECTED);
      var z = attributes.position.getZ(INTERSECTED);
      willRemovedPoints.push({ x, y, z });
    }
  } else if (INTERSECTED != null) {
    // resetColor(attributes, INTERSECTED);
    sphereInter.visible = false;
    INTERSECTED = null;
  }
}

function onWindowResize() {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
  controls.handleResize();
}

function animate() {
  requestAnimationFrame(animate);
  controls.update();
  renderer.render(scene, camera);
  stats.update();
}
