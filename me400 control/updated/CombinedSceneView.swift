import SwiftUI
import SceneKit
import Combine
import MetalKit
import ModelIO
import SceneKit.ModelIO

class CombinedSceneViewModel: ObservableObject {
    @Published var pipeLocation: Double
    @Published var magnification: Double = 1.0  // Changed default to 1.0
    
    // Pipe parameters
    @Published var pipe1XOffset: Double
    @Published var pipe1YLength: Double
    @Published var pipe1ZOffset: Double
    @Published var pipe1Diameter: Double
    
    @Published var pipe2XOffset: Double
    @Published var pipe2YLength: Double
    @Published var pipe2ZOffset: Double
    @Published var pipe2Diameter: Double
    
    @Published var pipe3XOffset: Double
    @Published var pipe3YLength: Double
    @Published var pipe3ZOffset: Double
    @Published var pipe3Diameter: Double
    
    // Aluminium parameters
    @Published var aluminiumXOffset: Double
    @Published var aluminiumYOffset: Double
    @Published var aluminiumZLength: Double
    @Published var aluminiumDiameter: Double
    
    // Bell parameters
    @Published var bellNeckLength: Double
    @Published var bellNeckDiameter: Double
    @Published var triggerPlacement: Double
    @Published var triggerDiameter: Double // New parameter for trigger sphere diameter
    @Published var bellConeHeight: Double   // New parameter for bell cone height
    @Published var bellConeBaseDiameter: Double // New parameter for bell cone base diameter
    
    @Published var hasDetection: Bool = false
    @Published var boundingBoxCenterX: Double = 0.0
    @Published var boundingBoxCenterY: Double = 0.0
    @Published var boundingBoxWidth: Double = 0.0
    @Published var boundingBoxHeight: Double = 0.0
    
    private var cancellables = Set<AnyCancellable>()
    
    init() {
        // Get default pipe location from ParameterManager but don't subscribe to changes
        let defaultPipeLocation = ParameterManager.shared.getParameter("DefaultPipeLocation", defaultValue: 1.5)
        pipeLocation = defaultPipeLocation
        
        // Pipe 1
        pipe1XOffset = ParameterManager.shared.getParameter("Pipe1XOffset", defaultValue: 0.0)
        pipe1YLength = ParameterManager.shared.getParameter("Pipe1YLength", defaultValue: 1.0)
        pipe1ZOffset = ParameterManager.shared.getParameter("Pipe1ZOffset", defaultValue: 0.0)
        pipe1Diameter = ParameterManager.shared.getParameter("Pipe1Diameter", defaultValue: 0.12)
        
        // Pipe 2
        pipe2XOffset = ParameterManager.shared.getParameter("Pipe2XOffset", defaultValue: 0.0)
        pipe2YLength = ParameterManager.shared.getParameter("Pipe2YLength", defaultValue: 1.0)
        pipe2ZOffset = ParameterManager.shared.getParameter("Pipe2ZOffset", defaultValue: 0.0)
        pipe2Diameter = ParameterManager.shared.getParameter("Pipe2Diameter", defaultValue: 0.08)
        
        // Pipe 3
        pipe3XOffset = ParameterManager.shared.getParameter("Pipe3XOffset", defaultValue: 0.0)
        pipe3YLength = ParameterManager.shared.getParameter("Pipe3YLength", defaultValue: 0.8)
        pipe3ZOffset = ParameterManager.shared.getParameter("Pipe3ZOffset", defaultValue: 0.0)
        pipe3Diameter = ParameterManager.shared.getParameter("Pipe3Diameter", defaultValue: 0.08)
        
        // Aluminium
        aluminiumXOffset = ParameterManager.shared.getParameter("AluminiumXOffset", defaultValue: 0.0)
        aluminiumYOffset = ParameterManager.shared.getParameter("AluminiumYOffset", defaultValue: 0.0)
        aluminiumZLength = ParameterManager.shared.getParameter("AluminiumZLength", defaultValue: 0.25)
        aluminiumDiameter = ParameterManager.shared.getParameter("AluminiumDiameter", defaultValue: 0.03)
        
        // Bell
        bellNeckLength = ParameterManager.shared.getParameter("BellNeckLength", defaultValue: 0.03)
        bellNeckDiameter = ParameterManager.shared.getParameter("BellNeckDiameter", defaultValue: 0.01)
        triggerPlacement = ParameterManager.shared.getParameter("TriggerPlacement", defaultValue: 0.05)
        triggerDiameter = ParameterManager.shared.getParameter("TriggerDiameter", defaultValue: 0.03)
        bellConeHeight = ParameterManager.shared.getParameter("BellConeHeight", defaultValue: 0.1)
        bellConeBaseDiameter = ParameterManager.shared.getParameter("BellConeBaseDiameter", defaultValue: 0.08)
        
        // Subscribe to ParameterManager changes, but ignore pipe location updates
        ParameterManager.shared.objectWillChange.sink { [weak self] in
            guard let self = self else { return }
            
            // Pipe 1
            self.pipe1XOffset = ParameterManager.shared.getParameter("Pipe1XOffset", defaultValue: 0.0)
            self.pipe1YLength = ParameterManager.shared.getParameter("Pipe1YLength", defaultValue: 1.0)
            self.pipe1ZOffset = ParameterManager.shared.getParameter("Pipe1ZOffset", defaultValue: 0.0)
            self.pipe1Diameter = ParameterManager.shared.getParameter("Pipe1Diameter", defaultValue: 0.12)
            
            // Pipe 2
            self.pipe2XOffset = ParameterManager.shared.getParameter("Pipe2XOffset", defaultValue: 0.0)
            self.pipe2YLength = ParameterManager.shared.getParameter("Pipe2YLength", defaultValue: 1.0)
            self.pipe2ZOffset = ParameterManager.shared.getParameter("Pipe2ZOffset", defaultValue: 0.0)
            self.pipe2Diameter = ParameterManager.shared.getParameter("Pipe2Diameter", defaultValue: 0.08)
            
            // Pipe 3
            self.pipe3XOffset = ParameterManager.shared.getParameter("Pipe3XOffset", defaultValue: 0.0)
            self.pipe3YLength = ParameterManager.shared.getParameter("Pipe3YLength", defaultValue: 0.8)
            self.pipe3ZOffset = ParameterManager.shared.getParameter("Pipe3ZOffset", defaultValue: 0.0)
            self.pipe3Diameter = ParameterManager.shared.getParameter("Pipe3Diameter", defaultValue: 0.08)
            
            // Aluminium
            self.aluminiumXOffset = ParameterManager.shared.getParameter("AluminiumXOffset", defaultValue: 0.0)
            self.aluminiumYOffset = ParameterManager.shared.getParameter("AluminiumYOffset", defaultValue: 0.0)
            self.aluminiumZLength = ParameterManager.shared.getParameter("AluminiumZLength", defaultValue: 0.25)
            self.aluminiumDiameter = ParameterManager.shared.getParameter("AluminiumDiameter", defaultValue: 0.03)
            
            // Bell
            self.bellNeckLength = ParameterManager.shared.getParameter("BellNeckLength", defaultValue: 0.03)
            self.bellNeckDiameter = ParameterManager.shared.getParameter("BellNeckDiameter", defaultValue: 0.01)
            self.triggerPlacement = ParameterManager.shared.getParameter("TriggerPlacement", defaultValue: 0.05)
            self.triggerDiameter = ParameterManager.shared.getParameter("TriggerDiameter", defaultValue: 0.03)
            self.bellConeHeight = ParameterManager.shared.getParameter("BellConeHeight", defaultValue: 0.1)
            self.bellConeBaseDiameter = ParameterManager.shared.getParameter("BellConeBaseDiameter", defaultValue: 0.08)
        }
        .store(in: &self.cancellables)
    }
}

struct CombinedSceneView: View {
    @ObservedObject private var viewModel: CombinedSceneViewModel
    @ObservedObject private var boundingBoxHandler = BoundingBoxHandler.shared
    @State private var scene: SCNScene?
    @State private var camera: SCNNode?
    @State private var lastPipeLocation: Double?
    @State private var displayLink: CADisplayLink?
    @State private var lastUpdateTime: TimeInterval = 0
    @StateObject private var cameraOffsetControl = CameraOffsetControl()
    
    private let initialPipeLocation: Double
    
    init(viewModel: CombinedSceneViewModel, initialPipeLocation: Double = 1.5) {
        self.viewModel = viewModel
        self.initialPipeLocation = initialPipeLocation
        
        // Create initial scene and camera
        let (scn, cam) = makeScene(
            viewModel: viewModel,
            pipeLocation: initialPipeLocation,
            serverManager: ServerCommunicationManager.shared,
            drawBoundingBox: false
        )
        _scene = State(initialValue: scn)
        _camera = State(initialValue: cam)
        _lastPipeLocation = State(initialValue: initialPipeLocation)
    }
    
    var body: some View {
        ZStack {
            if let scene = scene, let camera = camera {
                SceneView(
                    scene: scene,
                    pointOfView: camera,
                    options: [.allowsCameraControl, .autoenablesDefaultLighting]
                )
                .onAppear {
                    setupDisplayLink()
                }
                .onDisappear {
                    displayLink?.invalidate()
                    displayLink = nil
                }
//                .frame(height:400)
            }
            
            // Left side controls
            HStack {
//                Spacer()
                
                VStack(alignment: .center) {
                    Text("2.2")
                        .foregroundColor(.white)
                        .font(.caption)
                    
                    Slider(value: $viewModel.pipeLocation, in: 1.0...2.2)
                        .accentColor(.blue)
                        .rotationEffect(.degrees(-90))
                        .frame(width: 200, height: 200)
                    
                    Text("1.0")
                        .foregroundColor(.white)
                        .font(.caption)
                    
                    Text("Pipe Location")
                        .foregroundColor(.white)
                        .font(.caption2)
                        .padding(.top, 8)
                }
                .padding()
                .background(Color.black.opacity(0.5))
                .frame(width: 80)
                .clipped()
                .cornerRadius(8)
                
                Spacer()
                
                CameraOffsetControlView(control: cameraOffsetControl)
                    .padding()
                    .background(Color.black.opacity(0.5))
                    .cornerRadius(8)
                
//                Spacer()
            }
            .frame(maxWidth: .infinity, maxHeight: .infinity, alignment: .leading)
            .padding(.leading, 20)
            
            // Top debug overlay
            VStack {
                HStack {
                    Text("Pipe Location: \(viewModel.pipeLocation, specifier: "%.2f")")
                        .foregroundColor(.white)
                        .padding()
                        .background(Color.black.opacity(0.5))
                        .cornerRadius(8)
                    
                    // Tilt display
                    VStack(alignment: .leading, spacing: 2) {
                        Text("Tilt Values")
                            .foregroundColor(.white)
                            .font(.caption)
                        Text("Roll: \(ServerCommunicationManager.shared.currentRoll, specifier: "%.2f")°")
                            .foregroundColor(.cyan)
                            .font(.caption)
                        Text("Pitch: \(ServerCommunicationManager.shared.currentPitch, specifier: "%.2f")°")
                            .foregroundColor(.cyan)
                            .font(.caption)
                    }
                    .padding()
                    .background(Color.black.opacity(0.5))
                    .cornerRadius(8)
                    
                    Spacer()
                    
                    if boundingBoxHandler.hasDetection {
                        Text("Detection: Yes")
                            .foregroundColor(.green)
                            .padding()
                            .background(Color.black.opacity(0.5))
                            .cornerRadius(8)
                    } else {
                        Text("Detection: No")
                            .foregroundColor(.red)
                            .padding()
                            .background(Color.black.opacity(0.5))
                            .cornerRadius(8)
                    }
                    
                    Button(action: resetCamera) {
                        Text("Reset Camera")
                            .foregroundColor(.white)
                            .padding()
                            .background(Color.blue.opacity(0.7))
                            .cornerRadius(8)
                    }
                }
                .padding()
                
                Spacer()
            }
        }
    }
    
    private func resetCamera() {
        let currentLocation = viewModel.pipeLocation
        let targetLocation = 1.5
        
        // Calculate the adjustment needed
        let adjustment = currentLocation < targetLocation ? 0.1 : -0.1
        
        // Force scene update by setting lastPipeLocation to nil
        lastPipeLocation = nil
        
        // Apply the adjustment
        viewModel.pipeLocation = currentLocation + adjustment
        
        // Return to previous value
        viewModel.pipeLocation = currentLocation
    }
    
    private func setupDisplayLink() {
        displayLink = CADisplayLink(target: DisplayLinkTarget { [weak boundingBoxHandler] in
            guard let boundingBoxHandler = boundingBoxHandler else { return }
            
            // Update scene if needed
            if viewModel.pipeLocation != lastPipeLocation {
                let (newScene, newCamera) = makeScene(
                    viewModel: viewModel,
                    pipeLocation: viewModel.pipeLocation,
                    serverManager: ServerCommunicationManager.shared,
                    drawBoundingBox: boundingBoxHandler.hasDetection
                )
                scene = newScene
                camera = newCamera
                lastPipeLocation = viewModel.pipeLocation
            }
            
            // Update bounding box if needed
            if boundingBoxHandler.hasDetection, let currentScene = scene {
                removeBoundingBox(scene: currentScene)
                createBoundingBox(scene: currentScene, boundingBoxHandler: boundingBoxHandler)
            }
            
            // Update crosshair position if needed
            if let currentScene = scene {
                removeBoundingBox(scene: currentScene)
                if boundingBoxHandler.hasDetection {
                    createBoundingBox(scene: currentScene, boundingBoxHandler: boundingBoxHandler)
                }
            }
        }, selector: #selector(DisplayLinkTarget.update))
        
        displayLink?.add(to: .main, forMode: .common)
    }
}

// Helper class for CADisplayLink
private class DisplayLinkTarget {
    private let callback: () -> Void
    
    init(callback: @escaping () -> Void) {
        self.callback = callback
    }
    
    @objc func update() {
        callback()
    }
}

private func makeScene(viewModel: CombinedSceneViewModel, pipeLocation: CGFloat, serverManager: ServerCommunicationManager, drawBoundingBox: Bool = false) -> (scene: SCNScene, camera: SCNNode) {
    let scene = SCNScene()
    
    // Enable culling
    scene.isPaused = false
    scene.background.contents = UIColor.blue  // Set blue background
    
    // Add debug sphere at south pole only
    let southPole = createSphere(
        location: SCNVector3(0, -5, 0),  // South pole at y=-5
        diameter: 2.0,  // Radius 1.0
        color: .systemPink,
        opacity: 0.8
    )
    southPole.name = "southPole"
    
    scene.rootNode.addChildNode(southPole)
    
    // Create sky sphere
    let skySphere = SCNSphere(radius: 6)  // Reduced radius to 6
    let skyMaterial = SCNMaterial()
    
    // Create gradient for sky
    let gradientLayer = CAGradientLayer()
    gradientLayer.frame = CGRect(x: 0, y: 0, width: 1, height: 1)
    gradientLayer.colors = [
        UIColor(red: 0.7, green: 0.9, blue: 1.0, alpha: 1.0).cgColor,  // Lighter sky blue
        UIColor(red: 0.9, green: 0.95, blue: 1.0, alpha: 1.0).cgColor  // Very light blue
    ]
    gradientLayer.startPoint = CGPoint(x: 0.5, y: 0)
    gradientLayer.endPoint = CGPoint(x: 0.5, y: 1)
    
    let gradientImage = UIGraphicsImageRenderer(size: CGSize(width: 1, height: 1)).image { context in
        gradientLayer.render(in: context.cgContext)
    }
    
    skyMaterial.diffuse.contents = gradientImage
    skyMaterial.isDoubleSided = true
    skyMaterial.lightingModel = .constant
    skySphere.materials = [skyMaterial]
    
    let skyNode = SCNNode(geometry: skySphere)
    skyNode.name = "sky"
    scene.rootNode.addChildNode(skyNode)
    
    // Add main directional light (sun-like)
    let mainLight = SCNNode()
    mainLight.light = SCNLight()
    mainLight.light?.type = .directional
    mainLight.light?.color = UIColor(white: 1.0, alpha: 1.0)
    mainLight.light?.intensity = 1000
    mainLight.light?.castsShadow = true
    mainLight.light?.shadowRadius = 5
    mainLight.light?.shadowSampleCount = 32
    mainLight.light?.shadowBias = 0.005
    mainLight.light?.shadowMode = .deferred
    mainLight.position = SCNVector3(-3, 4, -2)  // Position for top-left lighting
    mainLight.eulerAngles = SCNVector3(Float.pi/6, -Float.pi/4, 0)  // Angle to light the bell
    scene.rootNode.addChildNode(mainLight)
    
    // Calculate all positions
    let pipe1Start = SCNVector3(0, 0, Float(-pipeLocation))
    
    let pipe1End = SCNVector3(
        Float(viewModel.pipe1XOffset),
        Float(viewModel.pipe1YLength),
        Float(-pipeLocation + viewModel.pipe1ZOffset)
    )
    
    let pipe2Start = pipe1End
    let pipe2End = SCNVector3(
        Float(viewModel.pipe1XOffset + viewModel.pipe2XOffset),
        Float(viewModel.pipe1YLength + viewModel.pipe2YLength),
        Float(-pipeLocation + viewModel.pipe1ZOffset + viewModel.pipe2ZOffset)
    )
    
    let pipe3Start = pipe2End
    let pipe3End = SCNVector3(
        Float(viewModel.pipe1XOffset + viewModel.pipe2XOffset + viewModel.pipe3XOffset),
        Float(viewModel.pipe1YLength + viewModel.pipe2YLength + viewModel.pipe3YLength),
        Float(-pipeLocation + viewModel.pipe1ZOffset + viewModel.pipe2ZOffset + viewModel.pipe3ZOffset)
    )
    
    let aluminiumStart = pipe3End
    let aluminiumEnd = SCNVector3(
        Float(viewModel.pipe1XOffset + viewModel.pipe2XOffset + viewModel.pipe3XOffset + viewModel.aluminiumXOffset),
        Float(viewModel.pipe1YLength + viewModel.pipe2YLength + viewModel.pipe3YLength + viewModel.aluminiumYOffset),
        Float(-pipeLocation + viewModel.pipe1ZOffset + viewModel.pipe2ZOffset + viewModel.pipe3ZOffset + viewModel.aluminiumZLength)
    )
    
    // Create all components
    let guideLine = createBox(
        startLocation: SCNVector3(0, -0.005, 1),
        endLocation: SCNVector3(0, -0.005, -2.5),
        width: 0.05,
        height: 0.01,
        color: .darkGray,
        opacity: 0.9
    )
    
    let pipe1 = createCylinder(
        startLocation: pipe1Start,
        endLocation: pipe1End,
        diameter: CGFloat(viewModel.pipe1Diameter),
        color: .brown,
        opacity: 0.9
    )
    pipe1.name = "pipe1"
    
    let pipe2 = createCylinder(
        startLocation: pipe2Start,
        endLocation: pipe2End,
        diameter: CGFloat(viewModel.pipe2Diameter),
        color: .brown,
        opacity: 0.9
    )
    pipe2.name = "pipe2"
    
    let pipe3 = createCylinder(
        startLocation: pipe3Start,
        endLocation: pipe3End,
        diameter: CGFloat(viewModel.pipe3Diameter),
        color: .brown,
        opacity: 0.9
    )
    pipe3.name = "pipe3"
    
    let aluminium = createCylinder(
        startLocation: aluminiumStart,
        endLocation: aluminiumEnd,
        diameter: CGFloat(viewModel.aluminiumDiameter),
        color: .gray,
        opacity: 0.9
    )
    
    // Create bell neck
    let bellNeckStart = aluminiumEnd
    let bellNeckEnd = SCNVector3(
        bellNeckStart.x,
        bellNeckStart.y - Float(viewModel.bellNeckLength),  // Use viewModel value
        bellNeckStart.z
    )
    
    let bellNeck = createCylinder(
        startLocation: bellNeckStart,
        endLocation: bellNeckEnd,
        diameter: CGFloat(viewModel.bellNeckDiameter),
        color: .lightGray,
        opacity: 0.9
    )
    bellNeck.name = "bellNeck"
    
    // Create bell cone
    let bellCone = createBellCone(
        startLocation: bellNeckEnd,
        height: viewModel.bellConeHeight,
        baseDiameter: viewModel.bellConeBaseDiameter,
        neckDiameter: viewModel.bellNeckDiameter
    )
    
    // Create trigger sphere with safety checks
    let triggerLocation = SCNVector3(
        bellNeckEnd.x,
        bellNeckEnd.y - Float(viewModel.triggerPlacement),  // Use viewModel value
        bellNeckEnd.z
    )
    
    let trigger = createSphere(
        location: triggerLocation,
        diameter: CGFloat(viewModel.triggerDiameter),
        color: .white,
        opacity: 1.0
    )
    trigger.name = "marker"
    
    // Add all components to scene
    scene.rootNode.addChildNode(guideLine)
    scene.rootNode.addChildNode(pipe1)
    scene.rootNode.addChildNode(pipe2)
    scene.rootNode.addChildNode(pipe3)
    scene.rootNode.addChildNode(aluminium)
    scene.rootNode.addChildNode(bellNeck)
    scene.rootNode.addChildNode(bellCone)
    scene.rootNode.addChildNode(trigger)
    
    // Create bounding box if needed
    if drawBoundingBox {
        createBoundingBox(scene: scene, boundingBoxHandler: serverManager.boundingBoxHandler)
    }
    else {
        removeBoundingBox(scene: scene)
    }
    
    // Add wireframe sphere
    let radius: Float = 5.0
    let sphere = SCNSphere(radius: CGFloat(radius))
    let material = SCNMaterial()
    material.diffuse.contents = UIColor.black
    material.transparency = 1.0
    material.fillMode = .lines
    material.lightingModel = .constant
    material.isDoubleSided = true
    sphere.materials = [material]
    let sphereNode = SCNNode(geometry: sphere)
    scene.rootNode.addChildNode(sphereNode)
    
    // Add latitude lines from equator (0°) to north pole (90°)
    for latitude in stride(from: 0, through: 90, by: 15) {
        let latitudeRad = Float(latitude) * Float.pi / 180
        let latitudeLine = createLatitudeLine(radius: radius, latitude: latitudeRad)
        scene.rootNode.addChildNode(latitudeLine)
    }
    
    // Add plus indicators at every 20 degrees
    let indicatorSize: Float = 0.2  // Size of the plus indicators
    
    // Create indicators from north pole to south pole
    for latitude in stride(from: 0, through: 180, by: 20) {
        let latitudeRad = Float(latitude) * Float.pi / 180.0
        
        // Calculate number of indicators at this latitude
        let numIndicators = Int(360.0 / 20.0)
        
        for i in 0..<numIndicators {
            let longitude = Float(i) * 20.0
            let longitudeRad = longitude * Float.pi / 180.0
            
            // Convert spherical to Cartesian coordinates
            let x = radius * sin(latitudeRad) * sin(longitudeRad)
            let y = radius * cos(latitudeRad)
            let z = radius * sin(latitudeRad) * cos(longitudeRad)
            
            // Create plus indicator with gray color and higher transparency
            let indicator = createPlusIndicator(
                location: SCNVector3(x, y, z),
                size: indicatorSize,
                color: .gray,
                opacity: 0.3
            )
            
            // Calculate rotation to face outward from sphere center
            let position = SCNVector3(x, y, z)
            let normal = normalize(position)
            let up = SCNVector3(0, 1, 0)
            let right = normalize(crossProduct(up, normal))
            let newUp = normalize(crossProduct(normal, right))
            
            let rotationMatrix = SCNMatrix4(
                m11: right.x, m12: right.y, m13: right.z, m14: 0,
                m21: newUp.x, m22: newUp.y, m23: newUp.z, m24: 0,
                m31: normal.x, m32: normal.y, m33: normal.z, m34: 0,
                m41: x, m42: y, m43: z, m44: 1
            )
            
            indicator.transform = rotationMatrix
            scene.rootNode.addChildNode(indicator)
        }
    }
    
    // Create camera after all scene elements are added
    let camera = SCNNode()
    let cameraComponent = SCNCamera()
    cameraComponent.usesOrthographicProjection = false
    cameraComponent.zNear = 0.1
    cameraComponent.zFar = 200.0
    cameraComponent.fieldOfView = 100
    cameraComponent.wantsHDR = false  // Disable HDR for better performance
    cameraComponent.wantsExposureAdaptation = false  // Disable exposure adaptation
    camera.camera = cameraComponent
    camera.name = "camera"
    
    // Set initial camera orientation if marker exists
    if let target = scene.rootNode.childNode(withName: "marker", recursively: true) {
        // Check for extreme target positions
        let maxHeight: Float = 5.0  // Maximum reasonable height
        let minHeight: Float = -5.0  // Minimum reasonable height
        
        // Clamp target height if needed
        let safeTarget = SCNVector3(
            target.position.x,
            max(minHeight, min(maxHeight, target.position.y)),  // Clamp y between minHeight and maxHeight
            target.position.z
        )
        
        // Calculate yaw (rotation around Y axis)
        let yaw = atan2(-safeTarget.x, -safeTarget.z)
        
        // Calculate pitch (rotation around X axis)
        let horizontalDistance = sqrt(safeTarget.x * safeTarget.x + safeTarget.z * safeTarget.z)
        let pitch = atan2(safeTarget.y, horizontalDistance)
        
        // Apply rotations with safety checks
        // Limit pitch to prevent extreme angles
        let safePitch = max(0, min(Float.pi, pitch))  // Limit to ±60 degrees
        camera.eulerAngles = SCNVector3(safePitch, yaw, 0)
    }
    scene.rootNode.addChildNode(camera)
    
    return (scene, camera)
}

private func removeBoundingBox(scene: SCNScene) {
    // Remove all bounding box related nodes
    scene.rootNode.childNodes.filter { 
        $0.name == "boundingBox" || 
        $0.name == "referenceBox" || 
        $0.name == "filteredBboxCenter" ||
        $0.name == "boundingBoxBorder" ||
        $0.name == "boxParent" ||
        $0.name == "filteredBoundBox" ||
        $0.name == "filteredBoundBoxBorder"
    }.forEach { $0.removeFromParentNode() }
}

private func createBoundingBox(scene: SCNScene, boundingBoxHandler: BoundingBoxHandler) {
    // Remove any existing bounding boxes
    removeBoundingBox(scene: scene)
    
    if !boundingBoxHandler.hasDetection {
        return
    }
    
    // Constants for box dimensions
    let refBoxWidth: CGFloat = 2.4
    let refBoxHeight: CGFloat = 1.35
    let refOriginDistance: Float = 1.5
    
    // Get marker position
    guard let marker = scene.rootNode.childNode(withName: "marker", recursively: true) else {
        return
    }
    
    // Create and normalize vector from origin to marker
    let originToMarker = marker.position
    let originToMarkerLength = sqrt(
        originToMarker.x * originToMarker.x +
        originToMarker.y * originToMarker.y +
        originToMarker.z * originToMarker.z
    )
    
    // Calculate new origin point
    let newOrigin = SCNVector3(
        originToMarker.x * refOriginDistance / originToMarkerLength,
        originToMarker.y * refOriginDistance / originToMarkerLength,
        originToMarker.z * refOriginDistance / originToMarkerLength
    )
    
    // Create a parent node at new origin
    let parentNode = SCNNode()
    parentNode.position = newOrigin
    parentNode.name = "boxParent"
    
    // Set up the parent node's orientation to face the marker
    let lookAtConstraint = SCNLookAtConstraint(target: marker)
    lookAtConstraint.isGimbalLockEnabled = true
    parentNode.constraints = [lookAtConstraint]
    
    // Create shared materials
    let refMaterial = SCNMaterial()
    refMaterial.diffuse.contents = UIColor(white: 0.9, alpha: 0.2)
    refMaterial.isDoubleSided = true
    refMaterial.lightingModel = .constant
    
    let axisMaterial = SCNMaterial()
    axisMaterial.diffuse.contents = UIColor.white
    axisMaterial.lightingModel = .constant
    
    let gridMaterial = SCNMaterial()
    gridMaterial.diffuse.contents = UIColor(white: 0.7, alpha: 0.5)
    gridMaterial.lightingModel = .constant
    
    let crosshairMaterial = SCNMaterial()
    crosshairMaterial.diffuse.contents = UIColor.red
    crosshairMaterial.lightingModel = .constant
    
    // Create reference box
    let refBox = SCNBox(width: refBoxWidth, height: refBoxHeight, length: 0.001, chamferRadius: 0)
    refBox.materials = [refMaterial]
    
    let refNode = SCNNode(geometry: refBox)
    refNode.name = "referenceBox"
    
    // Calculate the offset to center the reference box
    let refOffsetX = -Float(boundingBoxHandler.centerX) * Float(refBoxWidth/2)
    let refOffsetY = -Float(boundingBoxHandler.centerY) * Float(refBoxHeight/2)
    refNode.position = SCNVector3(refOffsetX, refOffsetY, 0)
    
    // Create coordinate axes
    let xAxis = SCNBox(width: refBoxWidth, height: 0.005, length: 0.002, chamferRadius: 0)
    xAxis.materials = [axisMaterial]
    let xNode = SCNNode(geometry: xAxis)
    xNode.position = SCNVector3(0, 0, 0.001)
    refNode.addChildNode(xNode)
    
    let yAxis = SCNBox(width: 0.005, height: refBoxHeight, length: 0.002, chamferRadius: 0)
    yAxis.materials = [axisMaterial]
    let yNode = SCNNode(geometry: yAxis)
    yNode.position = SCNVector3(0, 0, 0.001)
    refNode.addChildNode(yNode)
    
    // Add grid lines (horizontal and vertical)
    for i in -5...5 {
        let normalizedPos = Float(i) * 0.1
        
        // Horizontal line
        let hLine = SCNBox(width: refBoxWidth, height: 0.003, length: 0.002, chamferRadius: 0)
        hLine.materials = [gridMaterial]
        let hNode = SCNNode(geometry: hLine)
        hNode.position = SCNVector3(0, Float(normalizedPos) * Float(refBoxHeight), 0)
        refNode.addChildNode(hNode)
        
        // Vertical line
        let vLine = SCNBox(width: 0.003, height: refBoxHeight, length: 0.002, chamferRadius: 0)
        vLine.materials = [gridMaterial]
        let vNode = SCNNode(geometry: vLine)
        vNode.position = SCNVector3(Float(normalizedPos) * Float(refBoxWidth), 0, 0)
        refNode.addChildNode(vNode)
    }
    
    // Get offset values from ParameterManager
    let cameraOffsetX = ParameterManager.shared.getParameter("CameraOffsetX", defaultValue: 0.0)
    let cameraOffsetY = ParameterManager.shared.getParameter("CameraOffsetY", defaultValue: 0.0)
    
    // Calculate coordinate differences
    let bboxDiffX = boundingBoxHandler.centerX - cameraOffsetX
    let bboxDiffY = boundingBoxHandler.centerY - cameraOffsetY
    let filteredDiffX = boundingBoxHandler.filteredBboxCenterX - cameraOffsetX
    let filteredDiffY = boundingBoxHandler.filteredBboxCenterY - cameraOffsetY
    
    // Create text nodes for coordinate differences
    let textMaterial = SCNMaterial()
    textMaterial.diffuse.contents = UIColor.black
    textMaterial.lightingModel = .constant
    
    // Bounding box difference text
    let bboxText = SCNText(string: String(format: "BBox Δ: (%.4f, %.4f)", bboxDiffX, bboxDiffY), extrusionDepth: 0)
    bboxText.font = UIFont.systemFont(ofSize: 0.2)
    bboxText.materials = [textMaterial]
    let bboxTextNode = SCNNode(geometry: bboxText)
    bboxTextNode.position = SCNVector3(-1.3, -1.1, 0.002)
    bboxTextNode.scale = SCNVector3(0.5, 0.5, 0.5)
    bboxTextNode.name = "bboxDiffText"
    parentNode.addChildNode(bboxTextNode)
    
    // scene.rootNode.addChildNode(bboxTextNode)
    
    // Filtered box difference text
    let filteredText = SCNText(string: String(format: "Filter Δ: (%.4f, %.4f)", filteredDiffX, filteredDiffY), extrusionDepth: 0)
    filteredText.font = UIFont.systemFont(ofSize: 0.2)
    filteredText.materials = [textMaterial]
    let filteredTextNode = SCNNode(geometry: filteredText)
    filteredTextNode.position = SCNVector3(-1.3, -1.1+0.15, 0.002)
    filteredTextNode.scale = SCNVector3(0.5, 0.5, 0.5)
    filteredTextNode.name = "filteredDiffText"
    parentNode.addChildNode(filteredTextNode)
    // scene.rootNode.addChildNode(filteredTextNode)
    
    // Create crosshair with offset
    let crosshairSize: Float = 0.3  // Normalized units
    
    // Create crosshair container node
    let crosshairNode = SCNNode()
    crosshairNode.name = "crosshair"
    
    // Horizontal crosshair line
    let hCrosshair = SCNBox(width: CGFloat(crosshairSize), height: 0.008, length: 0.002, chamferRadius: 0)
    hCrosshair.materials = [crosshairMaterial]
    crosshairNode.addChildNode(SCNNode(geometry: hCrosshair))
    
    // Vertical crosshair line
    let vCrosshair = SCNBox(width: 0.008, height: CGFloat(crosshairSize), length: 0.002, chamferRadius: 0)
    vCrosshair.materials = [crosshairMaterial]
    crosshairNode.addChildNode(SCNNode(geometry: vCrosshair))
    
    // Position crosshair with offset
    crosshairNode.position = SCNVector3(
        Float(cameraOffsetX) * Float(refBoxWidth/2),
        Float(cameraOffsetY) * Float(refBoxHeight/2),
        0.002
    )
    refNode.addChildNode(crosshairNode)
    
    // Create filtered box
    let filteredBox = SCNBox(
        width: CGFloat(0.05 * 0.5625) * refBoxWidth,
        height: CGFloat(0.05) * refBoxHeight,
        length: 0.005,
        chamferRadius: 0
    )
    let filteredMaterial = SCNMaterial()
    filteredMaterial.diffuse.contents = UIColor.black
    filteredMaterial.transparency = 0.8
    filteredMaterial.isDoubleSided = true
    filteredMaterial.lightingModel = .constant
    filteredBox.materials = [filteredMaterial]
    
    let filteredNode = SCNNode(geometry: filteredBox)
    filteredNode.name = "flteredBoundingBox"
    
    // center white dot
    let filteredBoxCenter = SCNBox(
        width: CGFloat(0.01 * 0.5625) * refBoxWidth,
        height: CGFloat(0.01) * refBoxHeight,
        length: 0.006,
        chamferRadius: 0
    )
    let filteredCenterMaterial = SCNMaterial()
    filteredCenterMaterial.diffuse.contents = UIColor.white
    filteredCenterMaterial.transparency = 0.8
    filteredCenterMaterial.isDoubleSided = true
    filteredCenterMaterial.lightingModel = .constant
    filteredBoxCenter.materials = [filteredCenterMaterial]
    
    let filteredNodeCenter = SCNNode(geometry: filteredBoxCenter)
    filteredNodeCenter.name = "flteredBoundingBoxCenter"
    filteredNode.addChildNode(filteredNodeCenter)
    
    let filteredNodePositionX = Float(boundingBoxHandler.filteredBboxCenterX) * Float(refBoxWidth/2) + refOffsetX
    let filteredNodePositionY = Float(boundingBoxHandler.filteredBboxCenterY) * Float(refBoxHeight/2) + refOffsetY
    filteredNode.position = SCNVector3(
        filteredNodePositionX,
        filteredNodePositionY,
        0.015
    )
    
    // Create detection box
    let detBox = SCNBox(
        width: CGFloat(boundingBoxHandler.width) * refBoxWidth,
        height: CGFloat(boundingBoxHandler.height) * refBoxHeight,
        length: 0.003,
        chamferRadius: 0
    )
    
    let detMaterial = SCNMaterial()
    detMaterial.diffuse.contents = UIColor.orange
    detMaterial.transparency = 0.4
    detMaterial.isDoubleSided = true
    detMaterial.lightingModel = .constant
    detBox.materials = [detMaterial]
    
    let detNode = SCNNode(geometry: detBox)
    detNode.name = "boundingBox"
    
    // Add border to detection box
    let borderBox = SCNBox(
        width: CGFloat(boundingBoxHandler.width) * refBoxWidth,
        height: CGFloat(boundingBoxHandler.height) * refBoxHeight,
        length: 0.003,
        chamferRadius: 0
    )
    let borderMaterial = SCNMaterial()
    borderMaterial.diffuse.contents = UIColor.orange
    borderMaterial.transparency = 1.0
    borderMaterial.isDoubleSided = true
    borderMaterial.lightingModel = .constant
    borderMaterial.fillMode = .lines
    borderBox.materials = [borderMaterial]
    
    let borderNode = SCNNode(geometry: borderBox)
    borderNode.name = "boundingBoxBorder"
    detNode.addChildNode(borderNode)
    
    // Add boxes to parent node
    parentNode.addChildNode(filteredNode)
    parentNode.addChildNode(refNode)
    parentNode.addChildNode(detNode)
    
    // Add parent node to scene
    scene.rootNode.addChildNode(parentNode)
}

// MARK: - Helper Functions

private func createCylinder(startLocation: SCNVector3, endLocation: SCNVector3, diameter: CGFloat, color: UIColor, opacity: CGFloat) -> SCNNode {
    // Calculate direction vector
    let direction = SCNVector3(
        endLocation.x - startLocation.x,
        endLocation.y - startLocation.y,
        endLocation.z - startLocation.z
    )
    
    // Calculate length of the cylinder
    let length = sqrt(direction.x * direction.x + direction.y * direction.y + direction.z * direction.z)
    
    // Create cylinder (default orientation is along Y-axis)
    let cylinder = SCNCylinder(radius: diameter / 2, height: CGFloat(length))
    let material = SCNMaterial()
    material.diffuse.contents = color
    material.transparency = opacity
    material.isDoubleSided = true
    material.lightingModel = .constant
    material.fillMode = .fill
    cylinder.materials = [material]
    
    let node = SCNNode(geometry: cylinder)
    
    // Position at midpoint
    node.position = SCNVector3(
        (startLocation.x + endLocation.x) / 2,
        (startLocation.y + endLocation.y) / 2,
        (startLocation.z + endLocation.z) / 2
    )
    
    // Calculate rotation to align with direction
    if length > 0 {
        // Normalize direction vector
        let normalizedDirection = SCNVector3(
            direction.x / length,
            direction.y / length,
            direction.z / length
        )
        
        // Default cylinder direction is along Y-axis (0,1,0)
        let defaultDirection = SCNVector3(0, 1, 0)
        
        // Calculate rotation axis using cross product (right-hand rule)
        let rotationAxis = crossProduct(defaultDirection, normalizedDirection)
        
        // Calculate rotation angle using dot product
        let rotationAngle = acos(dotProduct(defaultDirection, normalizedDirection))
        
        // Apply rotation if axis is not zero
        if rotationAxis.x != 0 || rotationAxis.y != 0 || rotationAxis.z != 0 {
            node.rotation = SCNVector4(rotationAxis.x, rotationAxis.y, rotationAxis.z, rotationAngle)
        }
    }
    
    return node
}

// Helper functions for vector operations
private func crossProduct(_ a: SCNVector3, _ b: SCNVector3) -> SCNVector3 {
    return SCNVector3(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    )
}

private func dotProduct(_ a: SCNVector3, _ b: SCNVector3) -> Float {
    return a.x * b.x + a.y * b.y + a.z * b.z
}

private func createSphere(location: SCNVector3, diameter: CGFloat, color: UIColor, opacity: CGFloat) -> SCNNode {
    let sphere = SCNSphere(radius: diameter / 2)
    let material = SCNMaterial()
    material.diffuse.contents = color
    material.transparency = opacity
    material.isDoubleSided = true
    material.lightingModel = .constant  // Make it independent of lighting
    material.fillMode = .fill  // Solid fill
    sphere.materials = [material]
    
    let node = SCNNode(geometry: sphere)
    node.position = location
    return node
}

private func createLatitudeLine(radius: Float, latitude: Float) -> SCNNode {
    // Calculate the radius of the circle at this latitude
    let circleRadius = radius * cos(latitude)
    // Calculate the height (y position) of the circle
    let height = radius * sin(latitude)
    
    let circle = SCNTorus(ringRadius: CGFloat(circleRadius), pipeRadius: 0.005)
    let material = SCNMaterial()
    material.diffuse.contents = UIColor.green
    material.transparency = 0.5
    material.isDoubleSided = true
    material.lightingModel = .constant
    material.fillMode = .lines
    circle.materials = [material]
    
    let node = SCNNode(geometry: circle)
    // Position the circle at the correct height
    node.position.y = height
    return node
}

private func createBellCone(startLocation: SCNVector3, height: Double, baseDiameter: Double, neckDiameter: Double) -> SCNNode {
    let coneNode = SCNNode()
    let numCylinders = 20  // Number of cylinders in the circle
    let radius: Float = Float(baseDiameter)/2  // Radius of the circle
    let cylinderDiameter: CGFloat = 0.02  // Fixed diameter for all cylinders
    
    for i in 0..<numCylinders {
        let angle = Float(i) * (2.0 * Float.pi / Float(numCylinders))
        
        // Calculate end position using trigonometry
        let endX = startLocation.x + radius * cos(angle)
        let endY = startLocation.y - Float(0.1)  // Fixed height offset
        let endZ = startLocation.z + radius * sin(angle)
        
        // Create cylinder
        let cylinder = createCylinder(
            startLocation: startLocation,
            endLocation: SCNVector3(endX, endY, endZ),
            diameter: cylinderDiameter,
            color: .systemGray3,
            opacity: 0.9
        )
        
        coneNode.addChildNode(cylinder)
    }
    
    return coneNode
}

private func createPlusIndicator(location: SCNVector3, size: Float, color: UIColor = .red, opacity: CGFloat = 0.8) -> SCNNode {
    let plusNode = SCNNode()
    
    // Create horizontal line
    let horizontalLine = SCNBox(width: CGFloat(size), height: 0.01, length: 0.01, chamferRadius: 0)
    let material = SCNMaterial()
    material.diffuse.contents = color
    material.transparency = opacity
    material.isDoubleSided = true
    material.lightingModel = .constant
    horizontalLine.materials = [material]
    
    let horizontalNode = SCNNode(geometry: horizontalLine)
    plusNode.addChildNode(horizontalNode)
    
    // Create vertical line
    let verticalLine = SCNBox(width: 0.01, height: CGFloat(size), length: 0.01, chamferRadius: 0)
    verticalLine.materials = [material]
    
    let verticalNode = SCNNode(geometry: verticalLine)
    plusNode.addChildNode(verticalNode)
    
    plusNode.position = location
    return plusNode
}

private func createBox(startLocation: SCNVector3, endLocation: SCNVector3, width: CGFloat, height: CGFloat, color: UIColor, opacity: CGFloat) -> SCNNode {
    // Calculate direction vector
    let direction = SCNVector3(
        endLocation.x - startLocation.x,
        endLocation.y - startLocation.y,
        endLocation.z - startLocation.z
    )
    
    // Calculate length of the box
    let length = sqrt(direction.x * direction.x + direction.y * direction.y + direction.z * direction.z)
    
    // Create box (default orientation is along Z-axis)
    let box = SCNBox(width: CGFloat(width), height: CGFloat(height), length: CGFloat(length), chamferRadius: 0)
    let material = SCNMaterial()
    material.diffuse.contents = color
    material.transparency = opacity
    material.isDoubleSided = true
    material.lightingModel = .constant
    material.fillMode = .fill
    box.materials = [material]
    
    let node = SCNNode(geometry: box)
    
    // Position at midpoint
    node.position = SCNVector3(
        (startLocation.x + endLocation.x) / 2,
        (startLocation.y + endLocation.y) / 2,
        (startLocation.z + endLocation.z) / 2
    )
    
    // Calculate rotation to align with direction
    if length > 0 {
        // Normalize direction vector
        let normalizedDirection = SCNVector3(
            direction.x / length,
            direction.y / length,
            direction.z / length
        )
        
        // Default box direction is along Z-axis (0,0,1)
        let defaultDirection = SCNVector3(0, 0, 1)
        
        // Calculate rotation axis using cross product (right-hand rule)
        let rotationAxis = crossProduct(defaultDirection, normalizedDirection)
        
        // Calculate rotation angle using dot product
        let rotationAngle = acos(dotProduct(defaultDirection, normalizedDirection))
        
        // Apply rotation if axis is not zero
        if rotationAxis.x != 0 || rotationAxis.y != 0 || rotationAxis.z != 0 {
            node.rotation = SCNVector4(rotationAxis.x, rotationAxis.y, rotationAxis.z, rotationAngle)
        }
    }
    
    return node
}

// Helper function to normalize a vector
private func normalize(_ v: SCNVector3) -> SCNVector3 {
    let length = sqrt(v.x * v.x + v.y * v.y + v.z * v.z)
    if length > 0 {
        return SCNVector3(v.x / length, v.y / length, v.z / length)
    }
    return v
}

// MARK: - Preview

#Preview {
    CombinedSceneView(viewModel: CombinedSceneViewModel())
}
 
