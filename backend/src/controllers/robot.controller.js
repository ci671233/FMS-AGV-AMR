const RobotStatus = require('../models/robotStatus.model');
const Robot = require('../models/robot.model');

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
        await robot.save();

        // 로봇 상태 업데이트
        const robotStatus = await RobotStatus.findOne({ robotID });
        if (robotStatus) {
            robotStatus.location = targetLocation;
            robotStatus.status = 'moving';
            robotStatus.lastUpdated = new Date();
            await robotStatus.save();
        } else {
            await RobotStatus.create({
                robotID,
                status: 'moving',
                batteryLevel: robot.batteryLevel,
                location: targetLocation,
                lastUpdated: new Date()
            });
        }

        res.status(200).json({ message: 'Robot is moving', robot });
    } catch (error) {
        res.status(500).json({ message: 'Server error', error: error.message });
    }
};

// 로봇 상태조회
exports.getRobotStatus = async (req, res) => {
    try {
        const { robotID } = req.params;
        const robotStatus = await RobotStatus.findOne({ robotID });
        if (!robotStatus) {
            return res.status(404).json({ message: 'Robot status not found' });
        }
        res.status(200).json(robotStatus);
    } catch (error) {
        res.status(500).json({ message: 'Server error', error: error.message });
    }
};
