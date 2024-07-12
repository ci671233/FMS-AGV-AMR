const Robot = require('../models/robot.model');

// 로봇 등록 함수
exports.createRobot = async (req, res) => {
    try {
        const { name, model, batteryLevel, location } = req.body;

        const newRobot = new Factory({
            name,
            model,
            batteryLevel,
            location,
            status: 'idle' // 기본 상태를 'idle'으로 설정
        });
        await newRobot.save();

    } catch (error) {
        console.error('Error creating robot:', error);
        res.status(500).json({ message: 'Server error' });
    }
}

// 로봇 상태조회
exports.getRobotStatus = async (req, res) => {
    try {
        const { robotID } = req.params;
        const robot = await Robot.findById(robotID);
        if (!robot) {
            return res.status(404).json({ message: 'Robot status not found' });
        }
        res.status(200).json(robot);
    } catch (error) {
        res.status(500).json({ message: 'Server error', error: error.message });
    }
};


// 로봇 이동함수
exports.moveRobot = async (req, res) => {
    try {
        const { robotID } = req.params;
        const { targetLocation } = req.body;

        const robot = await Robot.findById(robotID);
        if (!robot) {
            return res.status(404).json({ message: 'Robot not found' });
        }

        // 로봇의 현재 위치 업데이트
        robot.location = targetLocation;
        robotStatus.status = 'moving';
        await robot.save();

        res.status(200).json({ message: 'Robot is moving', robot });
    } catch (error) {
        res.status(500).json({ message: 'Server error', error: error.message });
    }
};


