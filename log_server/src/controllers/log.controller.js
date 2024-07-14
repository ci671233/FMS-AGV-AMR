exports.getLatestLogs = async (req, res) => {
    // 하드코딩된 로그 데이터 (예시)
    const logs = [
        { id: 1, message: 'Robot 1 moved to position (50, 100)', timestamp: '2023-07-12 10:00:00' },
        { id: 2, message: 'Robot 2 battery low', timestamp: '2023-07-12 09:45:00' }
    ];
    res.json(logs);
};

