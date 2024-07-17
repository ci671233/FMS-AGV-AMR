const { exec } = require('child_process');
const fs = require('fs');
const path = require('path');

const convertPgmToPng = (pgmBuffer) => {
  return new Promise((resolve, reject) => {
    const pgmPath = path.join(__dirname, 'temp.pgm');
    const pngPath = path.join(__dirname, 'temp.png');

    fs.writeFileSync(pgmPath, pgmBuffer);

    exec(`pnmtopng ${pgmPath} > ${pngPath}`, (error) => {
      if (error) {
        return reject(error);
      }

      const pngBuffer = fs.readFileSync(pngPath);
      fs.unlinkSync(pgmPath);
      fs.unlinkSync(pngPath);
      resolve(pngBuffer);
    });
  });
};

module.exports = convertPgmToPng;





