import express from "express";
import { Server } from "socket.io";
import { pool } from "./db.js";
import http from "http";
import route from "./route/api.js"

const app = express();
const server = http.createServer(app);
const io = new Server(server);
const PORT = 3000;

// Cấu hình EJS làm view engine
app.set('view engine', 'ejs');

// Cho phép truy cập file tĩnh trong thư mục public
app.use(express.static('public'));

app.use(express.json({ limit: '5mb' }));

// Trả về lỗi rõ ràng nếu JSON không hợp lệ (ví dụ do payload bị hỏng)
app.use((err, req, res, next) => {
  if (err && err.type === 'entity.parse.failed') {
    return res.status(400).json({ error: 'JSON không hợp lệ. Vui lòng kiểm tra chuỗi JSON được gửi từ thiết bị.' });
  }
  next(err);
});


app.set("io", io);

app.use('/api', route)

app.get('/', (req, res) => {
  res.render("chart.ejs");
});

server.listen(PORT, () => console.log(`Server running on http://localhost:${PORT}`)); 
