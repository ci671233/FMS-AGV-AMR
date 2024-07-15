const axios = require('axios');

exports.getMonitoringData = async (req, res) => {
    try {
        const userId = req.user.id;

        // 선택된 맵 가져오기
        const mapResponse = await axios.get(`http://localhost:5557/map/selected`, {
            headers: { 'Authorization': req.headers.authorization }
        });
        const { mapId, mapImage, mapMeta } = mapResponse.data;

        // 로봇 위치 가져오기
        const robotsResponse = await axios.get(`http://localhost:5559/robot/positions`, {
            params: { mapId },
            headers: { 'Authorization': req.headers.authorization }
        });
        const robots = robotsResponse.data;

        // 최신 로그 가져오기
        const logsResponse = await axios.get(`http://localhost:5561/log/latest`, {
            headers: { 'Authorization': req.headers.authorization }
        });
        const logs = logsResponse.data;

        res.json({ mapImage, robots, logs });
    } catch (error) {
        res.status(500).json({ message: 'Error fetching monitoring data', error: error.message });
    }
};
