const gm = require('gm').subClass({ imageMagick: true });

async function convertPgmToPng(pgmBuffer) {
  return new Promise((resolve, reject) => {
    gm(pgmBuffer, 'image.pgm')
      .setFormat('png')
      .toBuffer((err, buffer) => {
        if (err) {
          reject(err);
        } else {
          resolve(buffer);
        }
      });
  });
}

module.exports = convertPgmToPng;


