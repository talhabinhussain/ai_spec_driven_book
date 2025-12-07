import React, { useEffect, useRef } from 'react';
import * as THREE from 'three';

const RobotViewer = ({
  title = "Humanoid Robot 3D Model",
  description = "Interactive 3D visualization of a humanoid robot"
}) => {
  const containerRef = useRef(null);
  const sceneRef = useRef(null);
  const cameraRef = useRef(null);
  const rendererRef = useRef(null);
  const robotRef = useRef(null);

  useEffect(() => {
    if (!containerRef.current) return;

    // Initialize Three.js scene
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0xf0f8ff); // Light blue background
    sceneRef.current = scene;

    // Create camera
    const camera = new THREE.PerspectiveCamera(
      75,
      containerRef.current.clientWidth / containerRef.current.clientHeight,
      0.1,
      1000
    );
    camera.position.z = 5;
    camera.position.y = 2;
    cameraRef.current = camera;

    // Create renderer
    const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    renderer.setSize(
      containerRef.current.clientWidth,
      containerRef.current.clientHeight
    );
    renderer.setPixelRatio(window.devicePixelRatio);
    rendererRef.current = renderer;

    // Add lighting
    const ambientLight = new THREE.AmbientLight(0x404040, 2);
    scene.add(ambientLight);

    const directionalLight = new THREE.DirectionalLight(0xffffff, 1);
    directionalLight.position.set(5, 5, 5);
    scene.add(directionalLight);

    const hemisphereLight = new THREE.HemisphereLight(0xffffbb, 0x080820, 1);
    scene.add(hemisphereLight);

    // Create a simple humanoid robot model
    const createRobot = () => {
      const robot = new THREE.Group();

      // Body (torso)
      const bodyGeometry = new THREE.CapsuleGeometry(0.5, 1, 4, 8);
      const bodyMaterial = new THREE.MeshPhongMaterial({ color: 0x4a90e2 }); // Blue
      const body = new THREE.Mesh(bodyGeometry, bodyMaterial);
      body.position.y = 1.5;
      robot.add(body);

      // Head
      const headGeometry = new THREE.SphereGeometry(0.6, 16, 16);
      const headMaterial = new THREE.MeshPhongMaterial({ color: 0xe0e0e0 }); // Light gray
      const head = new THREE.Mesh(headGeometry, headMaterial);
      head.position.y = 2.8;
      robot.add(head);

      // Eyes
      const eyeGeometry = new THREE.SphereGeometry(0.08, 16, 16);
      const eyeMaterial = new THREE.MeshPhongMaterial({ color: 0x000000 }); // Black

      const leftEye = new THREE.Mesh(eyeGeometry, eyeMaterial);
      leftEye.position.set(-0.2, 3.0, 0.5);
      robot.add(leftEye);

      const rightEye = new THREE.Mesh(eyeGeometry, eyeMaterial);
      rightEye.position.set(0.2, 3.0, 0.5);
      robot.add(rightEye);

      // Arms
      const armGeometry = new THREE.CapsuleGeometry(0.15, 1.2, 4, 8);
      const armMaterial = new THREE.MeshPhongMaterial({ color: 0x4a90e2 });

      const leftArm = new THREE.Mesh(armGeometry, armMaterial);
      leftArm.position.set(-1.2, 1.8, 0);
      leftArm.rotation.z = Math.PI / 6;
      robot.add(leftArm);

      const rightArm = new THREE.Mesh(armGeometry, armMaterial);
      rightArm.position.set(1.2, 1.8, 0);
      rightArm.rotation.z = -Math.PI / 6;
      robot.add(rightArm);

      // Legs
      const legGeometry = new THREE.CapsuleGeometry(0.2, 1.4, 4, 8);
      const legMaterial = new THREE.MeshPhongMaterial({ color: 0x357abd }); // Darker blue

      const leftLeg = new THREE.Mesh(legGeometry, legMaterial);
      leftLeg.position.set(-0.4, -0.7, 0);
      robot.add(leftLeg);

      const rightLeg = new THREE.Mesh(legGeometry, legMaterial);
      rightLeg.position.set(0.4, -0.7, 0);
      robot.add(rightLeg);

      return robot;
    };

    const robot = createRobot();
    robotRef.current = robot;
    scene.add(robot);

    // Add floor
    const floorGeometry = new THREE.PlaneGeometry(20, 20);
    const floorMaterial = new THREE.MeshPhongMaterial({
      color: 0xdddddd,
      side: THREE.DoubleSide
    });
    const floor = new THREE.Mesh(floorGeometry, floorMaterial);
    floor.rotation.x = -Math.PI / 2;
    floor.position.y = -1.5;
    scene.add(floor);

    // Add grid helper
    const gridHelper = new THREE.GridHelper(20, 20, 0x888888, 0x444444);
    scene.add(gridHelper);

    // Add axes helper
    const axesHelper = new THREE.AxesHelper(5);
    scene.add(axesHelper);

    // Add to DOM
    containerRef.current.appendChild(renderer.domElement);

    // Animation loop
    const animate = () => {
      requestAnimationFrame(animate);

      if (robotRef.current) {
        robotRef.current.rotation.y += 0.005; // Slow rotation
      }

      renderer.render(scene, camera);
    };

    animate();

    // Handle resize
    const handleResize = () => {
      if (containerRef.current && cameraRef.current && rendererRef.current) {
        cameraRef.current.aspect = containerRef.current.clientWidth / containerRef.current.clientHeight;
        cameraRef.current.updateProjectionMatrix();
        rendererRef.current.setSize(
          containerRef.current.clientWidth,
          containerRef.current.clientHeight
        );
      }
    };

    window.addEventListener('resize', handleResize);

    // Cleanup function
    return () => {
      window.removeEventListener('resize', handleResize);

      if (rendererRef.current && containerRef.current) {
        containerRef.current.removeChild(rendererRef.current.domElement);
      }

      if (rendererRef.current) {
        rendererRef.current.dispose();
      }
    };
  }, []);

  return (
    <div className="robot-viewer-container" style={{ width: '100%', height: '400px', position: 'relative' }}>
      <div
        ref={containerRef}
        className="robot-viewer"
        style={{ width: '100%', height: '100%', border: '1px solid #ddd', borderRadius: '8px' }}
        aria-label={title}
      />
      <div style={{ marginTop: '10px', textAlign: 'center', fontSize: '14px', color: '#666' }}>
        {description}
      </div>
    </div>
  );
};

export default RobotViewer;