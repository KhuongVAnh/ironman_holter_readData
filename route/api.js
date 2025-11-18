import express from "express";
import { pool } from "../db.js";
import fs from "fs";
import path from "path";


const router = express.Router();

let int = 0;
let time = Date.now();

router.post("/telemetry", async (req, res) => {
    try {
        let { ecg_signal, accel, gyro, sampling_rate } = req.body;

        // --- Parse JSON nếu bị stringify ---
        const parseIfString = (value, label) => {
            if (typeof value === "string") {
                try {
                    return JSON.parse(value.trim());
                } catch {
                    throw new Error(`${label} is not valid JSON`);
                }
            }
            return value;
        };

        ecg_signal = parseIfString(ecg_signal, "ecg_signal");
        accel = parseIfString(accel, "accel");
        gyro = parseIfString(gyro, "gyro");
        sampling_rate = parseIfString(sampling_rate, "sampling_rate");

        // --- Kiểm tra sampling_rate ---
        let sampling = {
            ecg_hz: sampling_rate?.ecg_hz ?? null,
            mpu_hz: sampling_rate?.mpu_hz ?? null,
            duration: sampling_rate?.duration ?? null,
        };
        if (!sampling.ecg_hz || !sampling.mpu_hz || !sampling.duration) {
            console.warn("⚠️ Missing sampling_rate fields");
        }

        // --- Kiểm tra dữ liệu sensor ---
        const validateAxis = (obj, label) => {
            if (
                !obj ||
                !Array.isArray(obj.x) ||
                !Array.isArray(obj.y) ||
                !Array.isArray(obj.z)
            ) {
                throw new Error(`${label} must contain x[], y[], z[] arrays`);
            }
            const len = Math.min(obj.x.length, obj.y.length, obj.z.length);
            return {
                x: obj.x.slice(0, len).map(Number),
                y: obj.y.slice(0, len).map(Number),
                z: obj.z.slice(0, len).map(Number),
            };
        };

        const accelClean = validateAxis(accel, "accel");
        const gyroClean = validateAxis(gyro, "gyro");

        // --- Làm sạch ecg_signal ---
        if (!Array.isArray(ecg_signal)) {
            throw new Error("ecg_signal must be an array");
        }
        const ecgClean = ecg_signal.map(Number).filter(Number.isFinite);

        // --- Phát realtime tới client ---
        const io = req.app.get("io");
        io.emit("new-ecg", {
            ecg_signal: ecgClean,
            accel: accelClean,
            gyro: gyroClean,
            sampling_rate: sampling,   // ✅ gửi cả tần số
        });

        console.log(
            `id: ${int++} time: ${Date.now() - time} 📡 Sent ${ecgClean.length} ECG @${sampling.ecg_hz}Hz, `
            + `${accelClean.x.length} accel @${sampling.mpu_hz}Hz, `
            + `${gyroClean.x.length} gyro (dur=${sampling.duration}s)`
        );
        time = Date.now();

        // // Lưu database (nếu cần)
        // await pool.query(
        //     `INSERT INTO Readings (ecg, giatoc, conquay, sampling) VALUES (?, ?, ?, ?)`,
        //     [
        //         JSON.stringify(ecgClean),
        //         JSON.stringify(accelClean),
        //         JSON.stringify(gyroClean),
        //         JSON.stringify(sampling),
        //     ]
        // );

        res.json({
            success: true,
            message: "✅ Data parsed & emitted successfully!",
            counts: {
                ecg: ecgClean.length,
                accel: accelClean.x.length,
                gyro: gyroClean.x.length,
            },
            sampling: sampling,
        });
    } catch (err) {
        console.error("❌ Error in /telemetry:", err);
        res.status(400).json({ success: false, error: err.message });
    }
});

// lưu database
// router.post("/save-reading", async (req, res) => {
//     const { ecg_signal, accel, gyro } = req.body;

//     try {
//         console.log("api save-reading is called!")

//         await pool.query(
//             `INSERT INTO Readings (ecg, giatoc, conquay) VALUES (?, ?, ?)`,
//             [JSON.stringify(ecg_signal), JSON.stringify(accel), JSON.stringify(gyro)]
//         );

//         res.json({ success: true });
//     } catch (err) {
//         console.error(err);
//         res.status(500).json({ success: false, error: err.message });
//     }
// });

// lưu vào file
const FILE_PATH = "readings.json";

/* 
 * Hàm stringify format object đẹp nhưng giữ mảng 1 dòng
 */
function stringifyFlatArrays(obj) {
    let json = JSON.stringify(obj, null, 2);

    // Gom mọi mảng số về 1 dòng: [1,2,3]
    json = json.replace(/\[\s+([\s\S]*?)\s+\]/g, (match, content) => {
        // Tách các phần tử, bỏ xuống dòng, bỏ khoảng trắng thừa
        let elements = content.split(/\s*,\s*/).map(x => x.trim());
        // Lọc phần tử rỗng
        elements = elements.filter(x => x.length > 0);
        return '[' + elements.join(', ') + ']';
    });

    return json;
}
let id = 0;
router.post("/save-reading", (req, res) => {
    const { ecg_signal, accel, gyro } = req.body;

    try {
        // 1) Nếu file chưa có → tạo mảng rỗng []
        if (!fs.existsSync(FILE_PATH)) {
            fs.writeFileSync(FILE_PATH, "[]");
        }

        // 2) Đọc file JSON
        const raw = fs.readFileSync(FILE_PATH, "utf8");
        let list = JSON.parse(raw);

        // 3) Tạo bản ghi mới
        id++;

        const reading = {
            id,
            timestamp: new Date().toISOString(),
            data: {
                ecg: ecg_signal,
                mpu: {
                    accel,
                    gyro
                }
            }
        };

        // 4) Thêm vào mảng
        list.push(reading);

        // 5) Stringify lại — giữ mảng 1 dòng
        const output = stringifyFlatArrays(list);

        // 6) Ghi file
        fs.writeFileSync(FILE_PATH, output);

        res.json({ success: true, id });
        console.log("lưu data thành công")
    } catch (err) {
        console.error(err);
        res.status(500).json({ success: false, error: err.message });
    }
});




export default router;
