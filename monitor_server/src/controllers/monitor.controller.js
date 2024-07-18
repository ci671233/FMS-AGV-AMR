const axios = require('axios');
const jwt = require('jsonwebtoken');

exports.getMonitoringData = async (req, res) => {
    try {
        const authHeader = req.headers['authorization'];
        const token = authHeader.split(' ')[1];
        const decoded = jwt.verify(token, process.env.JWT_SECRET_KEY);
        const userId = decoded.id;

        console.log('Fetching monitoring data for user:', userId);

        // 선택된 맵 가져오기
        const mapResponse = await axios.get(`http://localhost:5557/map/selected/${userId}`, {
            headers: { Authorization: authHeader }
        });
        const mapImage = mapResponse.data;
        console.log('Map image data:', mapImage);

        // 로봇 위치 가져오기
        const robotsResponse = await axios.get(`http://localhost:5559/robot/positions/${userId}`, {
            headers: { Authorization: authHeader }
        });
        const robot_position = robotsResponse.data;
        console.log('Robot positions data:', robot_position);

        // 최신 로그 가져오기
        const logsResponse = await axios.get('http://localhost:5561/log/latest', {
            headers: { Authorization: authHeader }
        });
        const log_top = logsResponse.data;
        console.log('Logs data:', log_top);
        
        res.json({ mapImage, robot_position, log_top });
    } catch (error) {
        console.error('Error fetching monitoring data:', error.message);
        res.status(500).json({ message: 'Error fetching monitoring data', error: error.message });
    }
};


