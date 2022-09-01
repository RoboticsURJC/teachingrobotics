import { yaml } from "js-yaml";
import { fs } from "fs";
let config = {};
let lineInterval, pointInterval, posInterval, objInterval;
let cont = 1;
// class obj3DPose {
// 	constructor(id,x,y,z,rx,ry,rz){
// 		this.id = id;
// 		this.x = x;
// 		this.y = y;
// 		this.z = z;
// 		this.rx = rx;
// 		this.ry = ry;
// 		this.rz = rz;
// 	}
// }

try {
  config = yaml.safeLoad(fs.readFileSync("public/config.yml", "utf8"));
} catch (e) {
  config.Server = "localhost";
  config.Port = "11000";
  config.updatePoints = 10;
  config.updateSegments = 10;
  config.linewidth = 2;
  config.pointsize = 1.5;
  config.spheresize = 0.35;
  config.camera = {};
  config.camera.x = 20;
  config.camera.y = 5;
  config.camera.z = 220;
}

export { lineInterval, posInterval, pointInterval, objInterval, cont, config };
