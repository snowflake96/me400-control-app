import Foundation
import Network
import Combine

// MARK: - Network Error Types
enum NetworkError: LocalizedError {
    case invalidPort
    case connectionTimeout
    case connectionFailed(String)
    case disconnected
    case sendFailed(String)
    case invalidPacketSize
    case decodingError
    
    var errorDescription: String? {
        switch self {
        case .invalidPort:
            return "Invalid port number"
        case .connectionTimeout:
            return "Connection timed out"
        case .connectionFailed(let reason):
            if reason.contains("Connection refused") {
                return "Server not available"
            }
            return "Connection failed: \(reason)"
        case .disconnected:
            return "Disconnected from server"
        case .sendFailed(let reason):
            return "Failed to send data: \(reason)"
        case .invalidPacketSize:
            return "Invalid packet size received"
        case .decodingError:
            return "Failed to decode packet"
        }
    }
}

// MARK: - Connection State
enum ConnectionState {
    case disconnected
    case connecting
    case connected
    case failed(NetworkError)
    
    var isConnected: Bool {
        if case .connected = self { return true }
        return false
    }
}

// MARK: - Network Configuration
struct NetworkConfiguration {
    let host: String
    let port: UInt16
    let connectionTimeout: TimeInterval
    let reconnectDelay: TimeInterval
    let maxReconnectAttempts: Int
    let keepaliveInterval: Int
    
    static let `default` = NetworkConfiguration(
        host: "100.102.243.9",
        port: 12345,
        connectionTimeout: 1.0,
        reconnectDelay: 0.5,
        maxReconnectAttempts: 20,
        keepaliveInterval: 5
    )
}

// MARK: - Network Manager Protocol
protocol NetworkManagerProtocol: AnyObject {
    var connectionState: AnyPublisher<ConnectionState, Never> { get }
    var receivedPackets: AnyPublisher<ReceivedPacket, Never> { get }
    
    func connect(configuration: NetworkConfiguration)
    func disconnect()
    func send(_ packet: Packet) async throws
}

// MARK: - Received Packet
struct ReceivedPacket {
    let type: PacketType
    let payload: Data
    let timestamp: Date
}

// MARK: - Network Manager Implementation
final class NetworkManager: NetworkManagerProtocol {
    // Published state
    private let connectionStateSubject = CurrentValueSubject<ConnectionState, Never>(.disconnected)
    private let receivedPacketsSubject = PassthroughSubject<ReceivedPacket, Never>()
    
    var connectionState: AnyPublisher<ConnectionState, Never> {
        connectionStateSubject.eraseToAnyPublisher()
    }
    
    var receivedPackets: AnyPublisher<ReceivedPacket, Never> {
        receivedPacketsSubject.eraseToAnyPublisher()
    }
    
    // Network components
    private var connection: NWConnection?
    private let queue = DispatchQueue(label: "com.me400.network", qos: .userInitiated)
    
    // Configuration
    private var configuration: NetworkConfiguration?
    
    // Connection state tracking
    private var isConnecting = false
    
    // Reconnection
    private var reconnectTimer: Timer?
    private var reconnectAttempts = 0
    
    // Connection timeout
    private var connectionTimeoutTimer: Timer?
    
    // MARK: - Connection Management
    
    func connect(configuration: NetworkConfiguration) {
        // Prevent multiple simultaneous connection attempts
        queue.async { [weak self] in
            guard let self = self else { return }
            
            // If already connecting or connected, ignore
            if self.isConnecting || self.connectionStateSubject.value.isConnected {
                print("NetworkManager: Already connecting or connected, ignoring connect request")
                return
            }
            
            // Cancel any existing connection
            if let existingConnection = self.connection {
                print("NetworkManager: Canceling existing connection")
                existingConnection.cancel()
                self.connection = nil
            }
            
            self.configuration = configuration
            self.reconnectAttempts = 0
            self.isConnecting = true
            
            self.establishConnection(configuration)
        }
    }
    
    private func establishConnection(_ config: NetworkConfiguration) {
        // Cancel any existing connection first
        if let existingConnection = connection {
            existingConnection.cancel()
            connection = nil
        }
        
        // Update state
        DispatchQueue.main.async {
            self.connectionStateSubject.send(.connecting)
        }
        
        // Configure TCP parameters
        let params = NWParameters.tcp
        // Remove interface requirement to allow any available interface
        // params.requiredInterfaceType = .other
        
        // Configure TCP options for keepalive
        if let tcpOptions = params.defaultProtocolStack.internetProtocol as? NWProtocolTCP.Options {
            tcpOptions.enableKeepalive = true
            tcpOptions.keepaliveIdle = config.keepaliveInterval
            tcpOptions.keepaliveInterval = config.keepaliveInterval
            tcpOptions.keepaliveCount = 3
        }
        
        // Create endpoint and connection
        let endpoint = NWEndpoint.hostPort(
            host: NWEndpoint.Host(config.host),
            port: NWEndpoint.Port(integerLiteral: config.port)
        )
        
        connection = NWConnection(to: endpoint, using: params)
        
        // Set up connection timeout
        setupConnectionTimeout(config.connectionTimeout)
        
        // Set up state handler
        connection?.stateUpdateHandler = { [weak self] state in
            self?.handleConnectionStateChange(state)
        }
        
        // Start connection
        connection?.start(queue: queue)
    }
    
    private func setupConnectionTimeout(_ timeout: TimeInterval) {
        connectionTimeoutTimer?.invalidate()
        connectionTimeoutTimer = Timer.scheduledTimer(withTimeInterval: timeout, repeats: false) { [weak self] _ in
            guard let self = self else { return }
            if !self.connectionStateSubject.value.isConnected {
                self.queue.async {
                    self.isConnecting = false
                    self.connection?.cancel()
                    
                    DispatchQueue.main.async {
                        self.connectionStateSubject.send(.failed(.connectionTimeout))
                    }
                    
                    // Attempt reconnection after timeout
                    self.attemptReconnection()
                }
            }
        }
    }
    
    private func handleConnectionStateChange(_ state: NWConnection.State) {
        // Cancel timeout timer
        connectionTimeoutTimer?.invalidate()
        
        print("NetworkManager: Connection state changed to: \(state)")
        
        switch state {
        case .ready:
            reconnectAttempts = 0
            isConnecting = false
            DispatchQueue.main.async {
                self.connectionStateSubject.send(.connected)
            }
            startReceiving()
            
        case .failed(let error):
            print("NetworkManager: Connection failed with error: \(error)")
            isConnecting = false
            let networkError = NetworkError.connectionFailed(error.localizedDescription)
            DispatchQueue.main.async {
                self.connectionStateSubject.send(.failed(networkError))
            }
            connection?.cancel()
            connection = nil
            attemptReconnection()
            
        case .cancelled:
            print("NetworkManager: Connection cancelled")
            isConnecting = false
            DispatchQueue.main.async {
                self.connectionStateSubject.send(.disconnected)
            }
            // Don't reconnect if explicitly cancelled
            
        case .waiting(let error):
            print("NetworkManager: Connection waiting with error: \(error)")
            // Network is temporarily unavailable, treat as failure to trigger reconnection
            isConnecting = false
            let networkError = NetworkError.connectionFailed(error.localizedDescription)
            DispatchQueue.main.async {
                self.connectionStateSubject.send(.failed(networkError))
            }
            connection?.cancel()
            connection = nil
            attemptReconnection()
            
        case .preparing:
            print("NetworkManager: Connection preparing")
            // Connection is being prepared, wait for next state
            
        default:
            break
        }
    }
    
    func disconnect() {
        queue.async { [weak self] in
            guard let self = self else { return }
            
            self.isConnecting = false
            self.reconnectAttempts = self.configuration?.maxReconnectAttempts ?? 0 // Prevent further reconnections
            self.reconnectTimer?.invalidate()
            self.reconnectTimer = nil
            self.connectionTimeoutTimer?.invalidate()
            self.connectionTimeoutTimer = nil
            self.connection?.cancel()
            self.connection = nil
            
            DispatchQueue.main.async {
                self.connectionStateSubject.send(.disconnected)
            }
        }
    }
    
    // MARK: - Reconnection
    
    private func attemptReconnection() {
        guard let config = configuration,
              reconnectAttempts < config.maxReconnectAttempts else {
            // Max attempts reached, reset connecting state
            print("NetworkManager: Max reconnection attempts reached (\(configuration?.maxReconnectAttempts ?? 0))")
            queue.async { [weak self] in
                self?.isConnecting = false
            }
            DispatchQueue.main.async { [weak self] in
                self?.connectionStateSubject.send(.disconnected)
            }
            return
        }
        
        reconnectAttempts += 1
        print("NetworkManager: Scheduling reconnection attempt \(reconnectAttempts)/\(config.maxReconnectAttempts) in \(config.reconnectDelay)s")
        
        // Schedule timer on main thread to ensure it works
        DispatchQueue.main.async { [weak self] in
            self?.reconnectTimer?.invalidate()
            self?.reconnectTimer = Timer.scheduledTimer(withTimeInterval: config.reconnectDelay, repeats: false) { _ in
                print("NetworkManager: Attempting reconnection \(self?.reconnectAttempts ?? 0)")
                self?.queue.async {
                    guard let self = self else { return }
                    // Reset connection state
                    self.connection = nil
                    self.isConnecting = false
                    // Establish new connection
                    self.establishConnection(config)
                }
            }
        }
    }
    
    // MARK: - Data Transmission
    
    func send(_ packet: Packet) async throws {
        guard let connection = connection,
              connectionStateSubject.value.isConnected else {
            throw NetworkError.disconnected
        }
        
        let data = packet.encode()
        
        return try await withCheckedThrowingContinuation { continuation in
            connection.send(content: data, completion: .contentProcessed { error in
                if let error = error {
                    continuation.resume(throwing: NetworkError.sendFailed(error.localizedDescription))
                } else {
                    continuation.resume()
                }
            })
        }
    }
    
    // MARK: - Data Reception
    
    private func startReceiving() {
        receiveNextPacket()
    }
    
    private func receiveNextPacket() {
        connection?.receive(
            minimumIncompleteLength: ProtocolConstants.packetSize,
            maximumLength: ProtocolConstants.packetSize
        ) { [weak self] data, _, isComplete, error in
            guard let self = self else { return }
            
            if let error = error {
                print("NetworkManager: Receive error: \(error)")
                DispatchQueue.main.async {
                    self.connectionStateSubject.send(.failed(.connectionFailed(error.localizedDescription)))
                }
                self.connection?.cancel()
                self.connection = nil
                self.attemptReconnection()
                return
            }
            
            if let data = data, data.count == ProtocolConstants.packetSize {
                self.processReceivedData(data)
            }
            
            if isComplete {
                print("NetworkManager: Connection closed by server")
                DispatchQueue.main.async {
                    self.connectionStateSubject.send(.disconnected)
                }
                self.connection = nil
                self.attemptReconnection()
                return
            }
            
            // Continue receiving
            self.receiveNextPacket()
        }
    }
    
    private func processReceivedData(_ data: Data) {
        guard let (type, payload) = PacketDecoder.decode(from: data) else {
            print("Failed to decode packet")
            return
        }
        
        print("Received packet: \(type.description)")
        
        let receivedPacket = ReceivedPacket(
            type: type,
            payload: payload,
            timestamp: Date()
        )
        
        DispatchQueue.main.async {
            self.receivedPacketsSubject.send(receivedPacket)
        }
    }
    
    deinit {
        disconnect()
    }
}

// MARK: - Network Manager Factory
enum NetworkManagerFactory {
    static func create() -> NetworkManagerProtocol {
        return NetworkManager()
    }
    
    #if DEBUG
    static func createMock() -> NetworkManagerProtocol {
        return MockNetworkManager()
    }
    #endif
}

// MARK: - Mock Network Manager for Testing
#if DEBUG
final class MockNetworkManager: NetworkManagerProtocol {
    private let connectionStateSubject = CurrentValueSubject<ConnectionState, Never>(.disconnected)
    private let receivedPacketsSubject = PassthroughSubject<ReceivedPacket, Never>()
    
    var connectionState: AnyPublisher<ConnectionState, Never> {
        connectionStateSubject.eraseToAnyPublisher()
    }
    
    var receivedPackets: AnyPublisher<ReceivedPacket, Never> {
        receivedPacketsSubject.eraseToAnyPublisher()
    }
    
    func connect(configuration: NetworkConfiguration) {
        connectionStateSubject.send(.connecting)
        
        DispatchQueue.main.asyncAfter(deadline: .now() + 0.5) {
            self.connectionStateSubject.send(.connected)
            self.simulateDataReception()
        }
    }
    
    func disconnect() {
        connectionStateSubject.send(.disconnected)
    }
    
    func send(_ packet: Packet) async throws {
        // Simulate sending
        try await Task.sleep(nanoseconds: 10_000_000) // 10ms
    }
    
    private func simulateDataReception() {
        Timer.scheduledTimer(withTimeInterval: 0.1, repeats: true) { _ in
            // Simulate receiving a bounding box packet
            let packet = ReceivedPacket(
                type: .bboxPos,
                payload: Data(count: 64),
                timestamp: Date()
            )
            self.receivedPacketsSubject.send(packet)
        }
    }
}
#endif 