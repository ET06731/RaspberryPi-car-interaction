/**
 * CarController - JavaScript frontend for RPi Smart Car Web Dashboard
 * Provides keyboard controls, touch controls, and sensor data polling
 */
class CarController {
    constructor() {
        this.apiBaseUrl = '/api';
        this.speed = 80;
        this.currentDirection = null;
        this.isConnected = true;
        this.pollingInterval = null;
        this.pollingDelay = 500; // 500ms
        
        // Key mappings for direction control
        this.keyMap = {
            'w': 'forward',
            'W': 'forward',
            'ArrowUp': 'forward',
            's': 'backward',
            'S': 'backward',
            'ArrowDown': 'backward',
            'a': 'left',
            'A': 'left',
            'ArrowLeft': 'left',
            'd': 'right',
            'D': 'right',
            'ArrowRight': 'right',
            'q': 'spin_left',
            'Q': 'spin_left',
            'e': 'spin_right',
            'E': 'spin_right',
            ' ': 'stop'
        };
        
        // Track currently pressed keys to prevent repeat actions
        this.pressedKeys = new Set();
        
        this.init();
    }
    
    /**
     * Initialize the controller
     */
    init() {
        this.bindElements();
        this.bindEvents();
        this.startSensorPolling();
        this.updateConnectionStatus(true);
        console.log('CarController initialized');
    }
    
    /**
     * Bind DOM elements to instance variables
     */
    bindElements() {
        this.speedSlider = document.getElementById('speedSlider');
        this.speedDisplay = document.getElementById('speedDisplay');
        this.speedValue = document.getElementById('speedValue');
        this.distanceValue = document.getElementById('distanceValue');
        this.connectionStatus = document.getElementById('connectionStatus');
        this.directionButtons = document.querySelectorAll('.btn-dir');
    }
    
    /**
     * Bind all event listeners
     */
    bindEvents() {
        // Speed slider control
        if (this.speedSlider) {
            this.speedSlider.addEventListener('input', (e) => this.handleSpeedChange(e));
        }
        
        // Direction button controls
        this.directionButtons.forEach(button => {
            const direction = button.dataset.direction;
            
            // Mouse/Touch events for buttons
            button.addEventListener('mousedown', (e) => this.handleButtonPress(e, direction));
            button.addEventListener('mouseup', (e) => this.handleButtonRelease(e, direction));
            button.addEventListener('mouseleave', (e) => this.handleButtonRelease(e, direction));
            
            // Touch events for mobile
            button.addEventListener('touchstart', (e) => {
                e.preventDefault();
                this.handleButtonPress(e, direction);
            });
            button.addEventListener('touchend', (e) => {
                e.preventDefault();
                this.handleButtonRelease(e, direction);
            });
        });
        
        // Keyboard controls
        document.addEventListener('keydown', (e) => this.handleKeyDown(e));
        document.addEventListener('keyup', (e) => this.handleKeyUp(e));
        
        // Prevent default behavior for game keys to avoid scrolling
        document.addEventListener('keydown', (e) => {
            if (this.keyMap[e.key]) {
                e.preventDefault();
            }
        });
    }
    
    /**
     * Handle speed slider change
     */
    handleSpeedChange(event) {
        this.speed = parseInt(event.target.value);
        
        if (this.speedDisplay) {
            this.speedDisplay.textContent = `${this.speed}%`;
        }
        if (this.speedValue) {
            this.speedValue.textContent = this.speed;
        }
        
        // Send speed update to backend
        this.setSpeed(this.speed);
    }
    
    /**
     * Handle direction button press
     */
    handleButtonPress(event, direction) {
        event.preventDefault();
        this.highlightButton(direction, true);
        this.sendCommand(direction, this.speed);
        this.currentDirection = direction;
    }
    
    /**
     * Handle direction button release
     */
    handleButtonRelease(event, direction) {
        event.preventDefault();
        this.highlightButton(direction, false);
        
        // Only stop if this was the current direction
        if (this.currentDirection === direction) {
            this.sendCommand('stop', 0);
            this.currentDirection = null;
        }
    }
    
    /**
     * Highlight or unhighlight a direction button
     */
    highlightButton(direction, active) {
        const button = document.querySelector(`.btn-dir[data-direction="${direction}"]`);
        if (button) {
            if (active) {
                button.classList.add('active');
            } else {
                button.classList.remove('active');
            }
        }
    }
    
    /**
     * Handle keyboard key down
     */
    handleKeyDown(event) {
        const key = event.key;
        
        // Check if this key is mapped and not already pressed
        if (this.keyMap[key] && !this.pressedKeys.has(key)) {
            this.pressedKeys.add(key);
            const direction = this.keyMap[key];
            
            this.highlightButton(direction, true);
            this.sendCommand(direction, this.speed);
            this.currentDirection = direction;
        }
    }
    
    /**
     * Handle keyboard key up
     */
    handleKeyUp(event) {
        const key = event.key;
        
        if (this.keyMap[key]) {
            this.pressedKeys.delete(key);
            const direction = this.keyMap[key];
            
            this.highlightButton(direction, false);
            
            // Only stop if no other keys are being held
            if (this.pressedKeys.size === 0) {
                this.sendCommand('stop', 0);
                this.currentDirection = null;
            }
        }
    }
    
    /**
     * Send command to Flask backend API
     */
    async sendCommand(direction, speed) {
        try {
            const response = await fetch(`${this.apiBaseUrl}/control`, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json'
                },
                body: JSON.stringify({
                    direction: direction,
                    speed: speed
                })
            });
            
            if (!response.ok) {
                throw new Error(`HTTP error! status: ${response.status}`);
            }
            
            const data = await response.json();
            
            if (data.status !== 'ok') {
                console.error('Command failed:', data);
                this.showError('命令执行失败');
            } else {
                this.updateConnectionStatus(true);
            }
            
            return data;
        } catch (error) {
            console.error('Failed to send command:', error);
            this.updateConnectionStatus(false);
            this.showError('连接失败，请检查网络');
            return null;
        }
    }
    
    /**
     * Set speed on the backend
     */
    async setSpeed(speed) {
        try {
            const response = await fetch(`${this.apiBaseUrl}/speed`, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json'
                },
                body: JSON.stringify({
                    speed: speed
                })
            });
            
            if (!response.ok) {
                throw new Error(`HTTP error! status: ${response.status}`);
            }
            
            const data = await response.json();
            
            if (data.status !== 'ok') {
                console.error('Speed set failed:', data);
            }
            
            return data;
        } catch (error) {
            console.error('Failed to set speed:', error);
            this.updateConnectionStatus(false);
            return null;
        }
    }
    
    /**
     * Fetch sensor data from backend
     */
    async fetchSensorData() {
        try {
            const response = await fetch(`${this.apiBaseUrl}/sensors`);
            
            if (!response.ok) {
                throw new Error(`HTTP error! status: ${response.status}`);
            }
            
            const data = await response.json();
            this.updateSensorDisplay(data);
            this.updateConnectionStatus(true);
            
            return data;
        } catch (error) {
            console.error('Failed to fetch sensor data:', error);
            this.updateConnectionStatus(false);
            return null;
        }
    }
    
    /**
     * Update sensor data display
     */
    updateSensorDisplay(data) {
        if (this.distanceValue && data.ultrasonic !== undefined) {
            this.distanceValue.textContent = data.ultrasonic.toFixed(1);
        }
    }
    
    /**
     * Start polling sensor data every 500ms
     */
    startSensorPolling() {
        // Initial fetch
        this.fetchSensorData();
        
        // Set up interval
        this.pollingInterval = setInterval(() => {
            this.fetchSensorData();
        }, this.pollingDelay);
    }
    
    /**
     * Stop sensor data polling
     */
    stopSensorPolling() {
        if (this.pollingInterval) {
            clearInterval(this.pollingInterval);
            this.pollingInterval = null;
        }
    }
    
    /**
     * Update connection status indicator
     */
    updateConnectionStatus(connected) {
        this.isConnected = connected;
        
        if (this.connectionStatus) {
            if (connected) {
                this.connectionStatus.textContent = '● 已连接';
                this.connectionStatus.classList.remove('disconnected');
                this.connectionStatus.classList.add('connected');
            } else {
                this.connectionStatus.textContent = '● 未连接';
                this.connectionStatus.classList.remove('connected');
                this.connectionStatus.classList.add('disconnected');
            }
        }
    }
    
    /**
     * Show error message to user
     */
    showError(message) {
        // Simple alert for now - could be enhanced with a toast notification
        console.error('CarController Error:', message);
        
        // Flash the status indicator to show error
        if (this.connectionStatus) {
            this.connectionStatus.style.color = '#ff4444';
            setTimeout(() => {
                if (this.isConnected) {
                    this.connectionStatus.style.color = '';
                }
            }, 1000);
        }
    }
    
    /**
     * Cleanup when controller is destroyed
     */
    destroy() {
        this.stopSensorPolling();
        this.pressedKeys.clear();
    }
}

// Initialize the controller when DOM is ready
document.addEventListener('DOMContentLoaded', () => {
    window.carController = new CarController();
});

// Cleanup on page unload
window.addEventListener('beforeunload', () => {
    if (window.carController) {
        window.carController.destroy();
    }
});
