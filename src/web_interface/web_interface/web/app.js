/**
 * Unitree Go2 Web Interface
 * Map visualization and robot control
 */

class Go2Interface {
    constructor() {
        this.canvas = document.getElementById('map-canvas');
        this.ctx = this.canvas.getContext('2d');

        this.mapImage = null;
        this.mapMetadata = null;
        this.robotImage = null;
        this.robotPosition = { x: 0, y: 0, theta: 0 };
        this.goalPosition = { x: null, y: null };
        this.trajectoryPoints = [];

        this.connectionStatusElement = document.getElementById('connection-status');
        this.cameraImageElement = document.getElementById('camera-feed');
        this.cameraStatusElement = document.getElementById('camera-status');
        this.cameraResolutionElement = document.getElementById('camera-resolution');
        this.cameraFetchInProgress = false;
        this.cameraIntervalId = null;

        this.currentMode = 'none';
        this.modeStatusElement = document.getElementById('mode-status');

        this.scale = 1.0;
        this.offsetX = 0;
        this.offsetY = 0;
        this.imageOffsetX = 0;
        this.imageOffsetY = 0;

        this.init();
    }

    async init() {
        this.setupEventListeners();
        await this.loadAvailableMaps();
        await this.loadRobotImage();
        await this.loadMap();
        await this.loadRobotPosition();
        await this.loadGoalPosition();
        await this.loadTrajectory();
        await this.loadMode();
        await this.loadObjectIds();
        this.startCameraUpdates();

        // Start update loop
        this.updateLoop();
    }

    setupEventListeners() {
        // Canvas click for setting goals
        this.canvas.addEventListener('click', (e) => this.handleCanvasClick(e));

        // Clear goal button
        document.getElementById('clear-goal-btn').addEventListener('click', () => this.clearGoal());

        // Map selector
        document.getElementById('map-selector').addEventListener('change', (e) => this.switchMap(e.target.value));

        // Mode buttons
        document.getElementById('simulation-mode-btn').addEventListener('click', () => this.setMode('simulation'));
        document.getElementById('real-mode-btn').addEventListener('click', () => this.setMode('real'));

        // Object IDs save button
        document.getElementById('save-object-ids-btn').addEventListener('click', () => this.saveObjectIds());

        // Window resize
        window.addEventListener('resize', () => this.resizeCanvas());

        // Setup panel resizing
        this.setupResizeHandles();
    }

    setupResizeHandles() {
        const horizontalResizer = document.getElementById('horizontal-resizer');
        const verticalResizer = document.getElementById('vertical-resizer');
        const mapPanel = document.getElementById('map-panel');
        const cameraPanel = document.getElementById('camera-panel');
        const infoPanel = document.getElementById('info-panel');
        const layout = document.querySelector('.layout');

        // Horizontal resizer (between map and side panels)
        let isResizingHorizontal = false;

        horizontalResizer.addEventListener('mousedown', (e) => {
            isResizingHorizontal = true;
            document.body.style.cursor = 'col-resize';
            e.preventDefault();
        });

        // Vertical resizer (between camera and info panels)
        let isResizingVertical = false;

        verticalResizer.addEventListener('mousedown', (e) => {
            isResizingVertical = true;
            document.body.style.cursor = 'row-resize';
            e.preventDefault();
        });

        document.addEventListener('mousemove', (e) => {
            if (isResizingHorizontal) {
                const layoutRect = layout.getBoundingClientRect();
                const newMapWidth = e.clientX - layoutRect.left - 20; // Account for padding
                const totalWidth = layoutRect.width - 40 - 20; // Subtract padding and resizer width
                const newMapFlex = (newMapWidth / totalWidth) * 3; // Scale to maintain proportions
                const newSideFlex = 3 - newMapFlex;

                if (newMapFlex > 0.5 && newSideFlex > 0.5) {
                    mapPanel.style.flex = newMapFlex;
                    document.querySelector('.side-panel').style.flex = newSideFlex;
                    this.resizeCanvas();
                }
            }

            if (isResizingVertical) {
                const sidePanelRect = document.querySelector('.side-panel').getBoundingClientRect();
                const newCameraHeight = e.clientY - sidePanelRect.top;
                const totalHeight = sidePanelRect.height - 20; // Subtract resizer height
                const newCameraFlex = newCameraHeight / totalHeight;
                const newInfoFlex = 1 - newCameraFlex;

                if (newCameraFlex > 0.2 && newInfoFlex > 0.2) {
                    cameraPanel.style.flex = newCameraFlex * 2; // Scale to maintain reasonable proportions
                    infoPanel.style.flex = newInfoFlex * 2;
                }
            }
        });

        document.addEventListener('mouseup', () => {
            if (isResizingHorizontal || isResizingVertical) {
                document.body.style.cursor = '';
                isResizingHorizontal = false;
                isResizingVertical = false;
            }
        });
    }

    async loadAvailableMaps() {
        try {
            const response = await fetch('/api/maps');
            const data = await response.json();

            if (data.success) {
                const selector = document.getElementById('map-selector');
                selector.innerHTML = '';

                data.maps.forEach(mapName => {
                    const option = document.createElement('option');
                    option.value = mapName;
                    option.textContent = mapName;
                    if (mapName === data.current) {
                        option.selected = true;
                    }
                    selector.appendChild(option);
                });
            }
        } catch (error) {
            console.error('Error loading maps:', error);
        }
    }

    async switchMap(mapName) {
        if (!mapName) return;

        try {
            const response = await fetch(`/api/maps/${mapName}`, {
                method: 'POST'
            });
            const data = await response.json();

            if (data.success) {
                await this.loadMap();
            }
        } catch (error) {
            console.error('Error switching map:', error);
        }
    }

    async loadMap() {
        document.getElementById('loading-overlay').style.display = 'flex';

        try {
            const response = await fetch('/api/map');
            const data = await response.json();

            if (data.success) {
                const img = new Image();
                img.onload = () => {
                    this.mapImage = img;
                    this.mapMetadata = data.map;
                    this.resizeCanvas();
                    this.updateMapInfo();
                    this.render();
                    document.getElementById('loading-overlay').style.display = 'none';
                };
                img.src = data.map.image;
            }
        } catch (error) {
            console.error('Error loading map:', error);
            document.getElementById('loading-overlay').style.display = 'none';
            this.showError('Failed to load map');
        }
    }

    async loadRobotImage() {
        return new Promise((resolve, reject) => {
            const img = new Image();
            img.onload = () => {
                this.robotImage = img;
                resolve();
            };
            img.onerror = (error) => {
                console.error('Error loading robot image:', error);
                resolve(); // Continue even if robot image fails to load
            };
            // Robot image path - the PNG should be in the web directory
            img.src = 'ChatGPT_Image_Oct_3__2025__11_46_27_PM-removebg-preview.png';
        });
    }

    async loadRobotPosition() {
        try {
            const response = await fetch('/api/position');
            const data = await response.json();

            if (data.success) {
                this.robotPosition = data.position;
                this.updateRobotDisplay();
                this.render();
                this.updateConnectionStatus(Boolean(data.connected));
            }
        } catch (error) {
            console.error('Error loading robot position:', error);
            this.updateConnectionStatus(false);
        }
    }

    async loadGoalPosition() {
        try {
            const response = await fetch('/api/goal');
            const data = await response.json();

            if (data.success) {
                this.goalPosition = data.goal;
                this.updateGoalDisplay();
                this.render();
            }
        } catch (error) {
            console.error('Error loading goal position:', error);
        }
    }

    async loadTrajectory() {
        if (!this.mapMetadata) return;

        try {
            const response = await fetch('/api/trajectory');
            const data = await response.json();

            if (data.success) {
                const points = (data.trajectory && data.trajectory.points) ? data.trajectory.points : [];
                this.trajectoryPoints = points;
                this.render();
            }
        } catch (error) {
            console.error('Error loading trajectory:', error);
        }
    }

    async loadCameraFrame() {
        if (!this.cameraImageElement || this.cameraFetchInProgress) return;

        this.cameraFetchInProgress = true;
        try {
            const response = await fetch('/api/camera', { cache: 'no-store' });
            const data = await response.json();

            if (data.success && data.image) {
                this.cameraImageElement.src = data.image;
                this.setCameraStatus('Streaming', true, data);
            } else {
                const message = data.error || 'Unavailable';
                this.setCameraStatus(message, false);
            }
        } catch (error) {
            console.error('Error loading camera frame:', error);
            this.setCameraStatus('Unavailable', false);
        } finally {
            this.cameraFetchInProgress = false;
        }
    }

    async setGoalPosition(x, y) {
        try {
            const response = await fetch('/api/goal', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ x, y })
            });

            const data = await response.json();
            if (data.success) {
                this.goalPosition = data.goal;
                this.updateGoalDisplay();
                this.render();
            }
        } catch (error) {
            console.error('Error setting goal position:', error);
        }
    }

    async clearGoal() {
        await this.setGoalPosition(null, null);
    }

    handleCanvasClick(event) {
        if (!this.mapMetadata) return;

        const rect = this.canvas.getBoundingClientRect();
        const canvasX = event.clientX - rect.left;
        const canvasY = event.clientY - rect.top;

        // Convert canvas coordinates to map coordinates (meters)
        const mapCoords = this.canvasToMap(canvasX, canvasY);

        this.setGoalPosition(mapCoords.x, mapCoords.y);
    }

    resizeCanvas() {
        if (!this.mapImage) return;

        const container = this.canvas.parentElement;
        const containerWidth = container.clientWidth;
        const containerHeight = container.clientHeight;

        // Make canvas fill the entire container
        this.canvas.width = containerWidth;
        this.canvas.height = containerHeight;

        // Calculate scale to fit image in canvas while maintaining aspect ratio
        const scaleX = containerWidth / this.mapImage.width;
        const scaleY = containerHeight / this.mapImage.height;
        this.scale = Math.min(scaleX, scaleY);

        // Calculate the scaled image dimensions
        const scaledWidth = this.mapImage.width * this.scale;
        const scaledHeight = this.mapImage.height * this.scale;

        // Center the image on the canvas
        this.imageOffsetX = (containerWidth - scaledWidth) / 2;
        this.imageOffsetY = (containerHeight - scaledHeight) / 2;

        this.render();
    }

    canvasToMap(canvasX, canvasY) {
        // Convert canvas pixel coordinates to map frame coordinates (meters)
        // Apply inverse 2D transformation: rotate and translate

        // Step 1: Remove image offset and convert from canvas to pixel coordinates
        const pixelX = (canvasX - this.imageOffsetX) / this.scale;
        const pixelY = (canvasY - this.imageOffsetY) / this.scale;

        // Step 2: Convert to meters (flip Y axis)
        const dx = pixelX * this.mapMetadata.resolution;
        const dy = (this.mapImage.height - pixelY) * this.mapMetadata.resolution;

        // Step 3: Apply forward rotation (inverse of the inverse rotation in mapToCanvas)
        const theta = this.mapMetadata.origin[2]; // Forward rotation
        const cosTheta = Math.cos(theta);
        const sinTheta = Math.sin(theta);
        const rotatedX = dx * cosTheta - dy * sinTheta;
        const rotatedY = dx * sinTheta + dy * cosTheta;

        // Step 4: Translate by origin offset
        const mapX = rotatedX + this.mapMetadata.origin[0];
        const mapY = rotatedY + this.mapMetadata.origin[1];

        return { x: mapX, y: mapY };
    }

    mapToCanvas(mapX, mapY) {
        // Convert map frame coordinates (meters) to canvas pixel coordinates
        // Apply 2D transformation: translate and rotate

        // Step 1: Translate by origin offset
        const dx = mapX - this.mapMetadata.origin[0];
        const dy = mapY - this.mapMetadata.origin[1];

        // Step 2: Apply rotation (inverse of map rotation)
        const theta = -this.mapMetadata.origin[2]; // Negative for inverse rotation
        const cosTheta = Math.cos(theta);
        const sinTheta = Math.sin(theta);
        const rotatedX = dx * cosTheta - dy * sinTheta;
        const rotatedY = dx * sinTheta + dy * cosTheta;

        // Step 3: Convert to pixel coordinates
        const pixelX = rotatedX / this.mapMetadata.resolution;
        const pixelY = this.mapImage.height - (rotatedY / this.mapMetadata.resolution);

        // Step 4: Apply canvas scale and image offset
        const canvasX = pixelX * this.scale + this.imageOffsetX;
        const canvasY = pixelY * this.scale + this.imageOffsetY;

        return { x: canvasX, y: canvasY };
    }

    render() {
        if (!this.mapImage || !this.mapMetadata) return;

        // Clear canvas
        this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);

        // Draw map centered on canvas with proper scaling
        const scaledWidth = this.mapImage.width * this.scale;
        const scaledHeight = this.mapImage.height * this.scale;
        this.ctx.drawImage(this.mapImage, this.imageOffsetX, this.imageOffsetY, scaledWidth, scaledHeight);

        // Draw trajectory before markers so it appears beneath them
        if (this.trajectoryPoints.length > 0) {
            this.drawTrajectory(this.trajectoryPoints);
        }

        // Draw goal position if set
        if (this.goalPosition.x !== null && this.goalPosition.y !== null) {
            this.drawGoal(this.goalPosition.x, this.goalPosition.y);
        }

        // Draw robot
        this.drawRobot(this.robotPosition.x, this.robotPosition.y, this.robotPosition.theta);
    }

    drawRobot(mapX, mapY, theta) {
        const pos = this.mapToCanvas(mapX, mapY);

        this.ctx.save();
        this.ctx.translate(pos.x, pos.y);

        // Adjust theta for map rotation
        // theta is in world frame, we need to account for map rotation
        const adjustedTheta = theta - this.mapMetadata.origin[2];
        this.ctx.rotate(-adjustedTheta); // Negative because canvas Y is inverted

        if (this.robotImage) {
            // Draw robot PNG image
            const imageSize = 80; // Size of the robot image in pixels

            // Rotate 90 degrees clockwise (π/2 radians)
            this.ctx.rotate(Math.PI / 2);

            this.ctx.drawImage(
                this.robotImage,
                -imageSize / 2,  // Center horizontally
                -imageSize / 2,  // Center vertically
                imageSize,
                imageSize
            );
        } else {
            // Fallback: Draw robot body (triangle pointing forward) if image not loaded
            const size = 20;
            this.ctx.beginPath();
            this.ctx.moveTo(size, 0);
            this.ctx.lineTo(-size/2, size);
            this.ctx.lineTo(-size/2, -size);
            this.ctx.closePath();

            this.ctx.fillStyle = '#3498db';
            this.ctx.fill();
            this.ctx.strokeStyle = '#2c3e50';
            this.ctx.lineWidth = 2;
            this.ctx.stroke();

            // Draw direction indicator
            this.ctx.beginPath();
            this.ctx.moveTo(0, 0);
            this.ctx.lineTo(size + 5, 0);
            this.ctx.strokeStyle = '#e74c3c';
            this.ctx.lineWidth = 3;
            this.ctx.stroke();
        }

        this.ctx.restore();

        // Draw position label
        this.ctx.fillStyle = '#2c3e50';
        this.ctx.font = '12px Arial';
        this.ctx.fillText(`(${mapX.toFixed(2)}, ${mapY.toFixed(2)})`, pos.x + 25, pos.y - 10);
    }

    drawTrajectory(points) {
        if (!this.mapMetadata || points.length === 0) return;

        this.ctx.save();
        this.ctx.strokeStyle = '#27ae60';
        this.ctx.lineWidth = Math.max(1.5, 2 * this.scale);
        this.ctx.lineJoin = 'round';
        this.ctx.lineCap = 'round';

        this.ctx.beginPath();
        points.forEach((point, index) => {
            const canvasPoint = this.mapToCanvas(point.x, point.y);
            if (index === 0) {
                this.ctx.moveTo(canvasPoint.x, canvasPoint.y);
            } else {
                this.ctx.lineTo(canvasPoint.x, canvasPoint.y);
            }
        });
        this.ctx.stroke();

        // Highlight starting point
        const first = this.mapToCanvas(points[0].x, points[0].y);
        this.ctx.fillStyle = '#2ecc71';
        this.ctx.beginPath();
        this.ctx.arc(first.x, first.y, 4, 0, 2 * Math.PI);
        this.ctx.fill();

        // Highlight last point if different
        if (points.length > 1) {
            const last = this.mapToCanvas(points[points.length - 1].x, points[points.length - 1].y);
            this.ctx.fillStyle = '#16a085';
            this.ctx.beginPath();
            this.ctx.arc(last.x, last.y, 4, 0, 2 * Math.PI);
            this.ctx.fill();
        }

        this.ctx.restore();
    }

    drawGoal(mapX, mapY) {
        const pos = this.mapToCanvas(mapX, mapY);
        const size = 15;

        // Draw target circles
        this.ctx.beginPath();
        this.ctx.arc(pos.x, pos.y, size, 0, 2 * Math.PI);
        this.ctx.strokeStyle = '#e74c3c';
        this.ctx.lineWidth = 3;
        this.ctx.stroke();

        this.ctx.beginPath();
        this.ctx.arc(pos.x, pos.y, size / 2, 0, 2 * Math.PI);
        this.ctx.strokeStyle = '#e74c3c';
        this.ctx.lineWidth = 2;
        this.ctx.stroke();

        // Draw crosshair
        this.ctx.beginPath();
        this.ctx.moveTo(pos.x - size - 5, pos.y);
        this.ctx.lineTo(pos.x + size + 5, pos.y);
        this.ctx.moveTo(pos.x, pos.y - size - 5);
        this.ctx.lineTo(pos.x, pos.y + size + 5);
        this.ctx.strokeStyle = '#e74c3c';
        this.ctx.lineWidth = 2;
        this.ctx.stroke();

        // Draw position label
        this.ctx.fillStyle = '#2c3e50';
        this.ctx.font = '12px Arial';
        this.ctx.fillText(`Goal: (${mapX.toFixed(2)}, ${mapY.toFixed(2)})`, pos.x + 25, pos.y + 20);
    }

    updateRobotDisplay() {
        document.getElementById('robot-x-display').textContent = `${this.robotPosition.x.toFixed(2)} m`;
        document.getElementById('robot-y-display').textContent = `${this.robotPosition.y.toFixed(2)} m`;
        document.getElementById('robot-theta-display').textContent = `${this.robotPosition.theta.toFixed(2)} rad`;
    }

    updateGoalDisplay() {
        const xDisplay = document.getElementById('goal-x-display');
        const yDisplay = document.getElementById('goal-y-display');

        if (this.goalPosition.x !== null && this.goalPosition.y !== null) {
            xDisplay.textContent = `${this.goalPosition.x.toFixed(2)} m`;
            yDisplay.textContent = `${this.goalPosition.y.toFixed(2)} m`;
        } else {
            xDisplay.textContent = '-';
            yDisplay.textContent = '-';
        }
    }

    updateMapInfo() {
        if (!this.mapMetadata) return;

        document.getElementById('map-resolution').textContent = `${this.mapMetadata.resolution} m/px`;
        document.getElementById('map-size').textContent = `${this.mapMetadata.width} × ${this.mapMetadata.height}`;
    }

    showError(message) {
        console.error(message);
        // Could add a toast notification here
    }

    async updateLoop() {
        // Periodically refresh robot and goal positions
        setInterval(async () => {
            await this.loadRobotPosition();
            await this.loadGoalPosition();
            await this.loadTrajectory();
            await this.loadMode();
        }, 1000); // Update every second
    }

    startCameraUpdates() {
        if (!this.cameraImageElement) return;

        this.loadCameraFrame();
        this.cameraIntervalId = setInterval(() => {
            this.loadCameraFrame();
        }, 250);
    }

    setCameraStatus(message, isConnected, metadata = null) {
        if (!this.cameraStatusElement) return;

        this.cameraStatusElement.textContent = message;
        this.cameraStatusElement.classList.remove('status-connected', 'status-disconnected');

        if (isConnected) {
            this.cameraStatusElement.classList.add('status-connected');
        } else if (message.toLowerCase().includes('unavailable') || message.toLowerCase().includes('error')) {
            this.cameraStatusElement.classList.add('status-disconnected');
        }

        if (this.cameraResolutionElement) {
            if (isConnected && metadata && metadata.width && metadata.height) {
                this.cameraResolutionElement.textContent = `${metadata.width}×${metadata.height}`;
                this.cameraResolutionElement.style.display = 'inline-block';
            } else {
                this.cameraResolutionElement.style.display = 'none';
            }
        }
    }

    updateConnectionStatus(isConnected) {
        if (!this.connectionStatusElement) return;

        this.connectionStatusElement.classList.remove('status-connected', 'status-disconnected');
        if (isConnected) {
            this.connectionStatusElement.textContent = 'Connected';
            this.connectionStatusElement.classList.add('status-connected');
        } else {
            this.connectionStatusElement.textContent = 'No ROS2 data';
            this.connectionStatusElement.classList.add('status-disconnected');
        }
    }

    async loadMode() {
        try {
            const response = await fetch('/api/mode');
            const data = await response.json();

            if (data.success) {
                this.currentMode = data.mode;
                this.updateModeDisplay();
            }
        } catch (error) {
            console.error('Error loading mode:', error);
        }
    }

    async setMode(mode) {
        try {
            const response = await fetch(`/api/set_mode/${mode}`, {
                method: 'POST'
            });
            const data = await response.json();

            if (data.success) {
                this.currentMode = data.mode;
                this.updateModeDisplay();
                alert(data.message || `${mode} mode activated`);
            } else {
                alert(`Failed to set ${mode} mode: ${data.error}`);
            }
        } catch (error) {
            console.error('Error setting mode:', error);
            alert(`Error setting ${mode} mode: ${error.message}`);
        }
    }

    updateModeDisplay() {
        if (!this.modeStatusElement) return;

        // Update text
        const modeText = this.currentMode.charAt(0).toUpperCase() + this.currentMode.slice(1);
        this.modeStatusElement.textContent = modeText;

        // Update button states
        const simulationBtn = document.getElementById('simulation-mode-btn');
        const realBtn = document.getElementById('real-mode-btn');

        simulationBtn.classList.remove('active');
        realBtn.classList.remove('active');

        if (this.currentMode === 'simulation') {
            simulationBtn.classList.add('active');
        } else if (this.currentMode === 'real') {
            realBtn.classList.add('active');
        }
    }

    async loadObjectIds() {
        try {
            const response = await fetch('/api/object_ids');
            const data = await response.json();

            if (data.success && data.object_ids) {
                const textarea = document.getElementById('object-ids-input');
                textarea.value = data.object_ids.join('\n');
            }
        } catch (error) {
            console.error('Error loading object IDs:', error);
        }
    }

    async saveObjectIds() {
        try {
            const textarea = document.getElementById('object-ids-input');
            const objectIds = textarea.value
                .split('\n')
                .map(line => line.trim())
                .filter(line => line.length > 0);

            const response = await fetch('/api/object_ids', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ object_ids: objectIds })
            });

            const data = await response.json();
            if (data.success) {
                alert('Object IDs saved successfully!');
            } else {
                alert('Failed to save object IDs: ' + (data.error || 'Unknown error'));
            }
        } catch (error) {
            console.error('Error saving object IDs:', error);
            alert('Error saving object IDs: ' + error.message);
        }
    }
}

// Initialize the interface when DOM is ready
document.addEventListener('DOMContentLoaded', () => {
    const go2Interface = new Go2Interface();
});
