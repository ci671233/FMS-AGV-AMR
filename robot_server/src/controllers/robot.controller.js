const Robot = require('../models/robot.model');

exports.getRobotPositions = async (req, res) => {
    const mapId = req.query.mapId; // 클라이언트에서 전달한 맵 ID

    // 하드코딩된 로봇 위치 데이터 (예시)
    const robotPositions = [
        { id: 1, name: 'Robot 1', x: 50, y: 100, model: 'Model X', battery: 80, status: 'active', mapId },
        { id: 2, name: 'Robot 2', x: 150, y: 200, model: 'Model Y', battery: 60, status: 'idle', mapId }
    ];

    res.json(robotPositions);
};

exports.getRobotById = async (req, res) => {
    try {
        const robotId = req.params.id;

        // 하드코딩된 로봇 데이터 (예시)
        const robot = { id: robotId, name: 'Robot 1', x: 50, y: 100, model: 'Model X', battery: 80, status: 'active' };

        if (!robot) {
            return res.status(404).json({ message: 'Robot not found' });
        }

        res.json(robot);
    } catch (error) {
        res.status(500).json({ message: 'Error fetching robot data', error: error.message });
    }
};
