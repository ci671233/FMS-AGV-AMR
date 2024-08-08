const Robot = require('../models/robot.model');

exports.getPositions = async (req, res) => {
  try {
    const robotPositions = await getRobotPositions();
    res.json(robotPositions);
  } catch (error) {
    console.error('Error fetching robot positions:', error);
    res.status(500).json({ message: 'Error fetching robot positions' });
  }
};

async function getRobotPositions() {
  try {
    const robots = await Robot.find().select('name location');
    return robots;
  } catch (error) {
    console.error('Error fetching robot positions:', error);
    return [];
  }
}

