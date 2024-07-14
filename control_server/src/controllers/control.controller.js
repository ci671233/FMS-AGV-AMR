const axios = require('axios');

exports.controlRobot = async (req, res) => {
    try {
        const robotId = req.params.id;
        const { position } = req.body;

        // 로봇 정보를 robot_server에서 가져오기
        const robotResponse = await axios.get(`http://localhost:5559/robots/${robotId}`, {
            headers: { 'Authorization': req.headers.authorization }
        });
        const robot = robotResponse.data;

        if (!robot) {
            return res.status(404).json({ message: 'Robot not found' });
        }

        // 로봇에게 실제로 이동 명령을 전달하는 로직이 추가되어야 합니다 (MQTT, WebSocket 등)

        // 예시로 이동 명령을 성공적으로 수행했다고 가정
        const updatedRobot = { ...robot, position };

        res.json(updatedRobot);
    } catch (error) {
        console.error('Error controlling robot:', error);
        res.status(500).json({ message: 'Server error', error: error.message });
    }
};
