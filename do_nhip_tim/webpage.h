const char webpageHTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<title>UTC - Heart & SpO₂ Monitor</title>
<script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
<link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.0.0/css/all.min.css">

<style>
  /* === Giao diện chung === */
  body { font-family: 'Segoe UI', Arial, sans-serif; text-align: center; background: #0b0c10; color: #fff; margin: 0; padding: 20px; }
  
  /* === Header: Logo & Tên trường === */
  .header { margin-bottom: 30px; border-bottom: 2px solid #1f2833; padding-bottom: 20px; }
  .logo { width: 100px; height: auto; margin-bottom: 10px; }
  .uni-name { font-size: 1.2rem; font-weight: bold; color: #fff; text-transform: uppercase; letter-spacing: 1px; margin: 5px 0; }
  .project-name { font-size: 1.5rem; color: #66fcf1; margin-top: 10px; }

  /* === Container chứa biểu đồ === */
  .container { display: flex; justify-content: center; gap: 20px; flex-wrap: wrap; max-width: 1200px; margin: 0 auto; }
  .card { 
    background: #1f2833; 
    padding: 20px; 
    border-radius: 15px; 
    box-shadow: 0 4px 15px rgba(0,0,0,0.5); 
    flex: 1;
    min-width: 300px; 
    border: 1px solid #333;
  }
  
  h2 { color: #45a29e; margin-top: 0; font-size: 1.1rem; }
  canvas { width: 100% !important; height: 250px !important; background: #000; border-radius: 8px; border: 1px solid #444; }
  .value { font-size: 2.5em; font-weight: bold; margin: 10px 0 20px 0; color: #fff; text-shadow: 0 0 10px rgba(102, 252, 241, 0.5); }
  .unit { font-size: 0.4em; color: #aaa; }

  /* === Khu vực nút bấm === */
  .controls { margin-top: 30px; display: flex; justify-content: center; gap: 20px; }
  .btn {
    border: none;
    padding: 12px 25px;
    border-radius: 50px;
    font-size: 1rem;
    cursor: pointer;
    font-weight: bold;
    transition: all 0.3s ease;
    display: flex;
    align-items: center;
    gap: 8px;
  }
  
  .btn-reset { background-color: #c5c6c7; color: #0b0c10; }
  .btn-reset:hover { background-color: #fff; transform: scale(1.05); }

  .btn-download { background-color: #45a29e; color: #0b0c10; }
  .btn-download:hover { background-color: #66fcf1; transform: scale(1.05); box-shadow: 0 0 15px #66fcf1; }

</style>
</head>
<body>

  <div class="header">
    <img src="https://inkythuatso.com/uploads/thumbnails/800/2021/12/logo-dai-hoc-giao-thong-van-tai-inkythuatso-01-23-16-23-19.jpg" alt="UTC Logo" class="logo">
    <div class="uni-name">TRƯỜNG ĐẠI HỌC GIAO THÔNG VẬN TẢI</div>
    <h1 class="project-name">HỆ THỐNG GIÁM SÁT SỨC KHỎE TỪ XA</h1>
  </div>

  <div class="container">
    <div class="card">
      <h2><i class="fas fa-heartbeat"></i> NHỊP TIM (HEART RATE)</h2>
      <div class="value" id="bpmVal">-- <span class="unit">BPM</span></div>
      <div style="position: relative; height: 250px;">
          <canvas id="chartIR"></canvas>
      </div>
    </div>

    <div class="card">
      <h2><i class="fas fa-lungs"></i> NỒNG ĐỘ OXY (SpO₂)</h2>
      <div class="value" id="spo2Val" style="color: #ff5c5c; text-shadow: 0 0 10px rgba(255, 92, 92, 0.5);">-- <span class="unit">%</span></div>
      <div style="position: relative; height: 250px;">
          <canvas id="chartSpO2"></canvas>
      </div>
    </div>
  </div>

  <div class="controls">
    <button class="btn btn-reset" onclick="resetData()">
      <i class="fas fa-redo"></i> Reset Dữ Liệu
    </button>
    <button class="btn btn-download" onclick="downloadCSV()">
      <i class="fas fa-file-excel"></i> Tải về Excel/Google Sheet
    </button>
  </div>

<script>
// === Biến lưu trữ toàn bộ dữ liệu để tải về ===
let recordedData = []; // Mảng chứa {time, bpm, spo2, ir}

// === Cấu hình Biểu đồ IR ===
let irCtx = document.getElementById('chartIR').getContext('2d');
let irChart = new Chart(irCtx, {
  type: 'line',
  data: {
    labels: [],
    datasets: [{
      label: 'Tín hiệu hồng ngoại (mV)',
      data: [],
      borderColor: '#66fcf1',
      backgroundColor: 'rgba(102, 252, 241, 0.1)',
      borderWidth: 2,
      fill: true,
      tension: 0.4,
      pointRadius: 0
    }]
  },
  options: {
    responsive: true,
    maintainAspectRatio: false,
    animation: false,
    interaction: { intersect: false },
    scales: {
      x: { display: false },
      y: { display: true, grid: { color: '#333' }, ticks: { color: '#888' } }
    },
    plugins: { legend: { display: false } }
  }
});

// === Cấu hình Biểu đồ SpO2 ===
let spo2Ctx = document.getElementById('chartSpO2').getContext('2d');
let spo2Chart = new Chart(spo2Ctx, {
  type: 'line',
  data: {
    labels: [],
    datasets: [{
      label: 'SpO₂ (%)',
      data: [],
      borderColor: '#ff5c5c',
      borderWidth: 2,
      fill: false,
      tension: 0.1,
      pointRadius: 0
    }]
  },
  options: {
    responsive: true,
    maintainAspectRatio: false,
    animation: false,
    scales: {
      x: { display: false },
      y: { min: 80, max: 100, grid: { color: '#333' }, ticks: { color: '#888', stepSize: 5 } }
    },
    plugins: { legend: { display: false } }
  }
});

// === Hàm cập nhật dữ liệu (50ms) ===
let timeCounter = 0;
const maxDisplayPoints = 150; // Số điểm hiển thị trên đồ thị

setInterval(() => {
  fetch('/data')
    .then(r => r.json())
    .then(data => {
      // 1. Cập nhật giao diện số
      document.getElementById("bpmVal").innerHTML = data.bpm + ' <span class="unit">BPM</span>';
      document.getElementById("spo2Val").innerHTML = data.spo2 + ' <span class="unit">%</span>';

      let currentTimeStr = timeCounter.toFixed(2);

      // 2. Lưu vào mảng tổng (để tải về sau này)
      // Chỉ lưu khi có dữ liệu thực (tránh lưu hàng ngàn dòng rỗng lúc chưa đo)
      if (data.ir > 10 || data.bpm > 0) {
        recordedData.push({
            time: currentTimeStr,
            bpm: data.bpm,
            spo2: data.spo2,
            ir: data.ir
        });
      }

      // 3. Cập nhật đồ thị (chỉ giữ 150 điểm để web mượt)
      irChart.data.labels.push(currentTimeStr);
      irChart.data.datasets[0].data.push(data.ir);

      spo2Chart.data.labels.push(currentTimeStr);
      spo2Chart.data.datasets[0].data.push(data.spo2);

      if (irChart.data.labels.length > maxDisplayPoints) {
        irChart.data.labels.shift();
        irChart.data.datasets[0].data.shift();
        spo2Chart.data.labels.shift();
        spo2Chart.data.datasets[0].data.shift();
      }

      irChart.update();
      spo2Chart.update();

      timeCounter += 0.05; // Tăng thời gian (khớp với setInterval 50ms)
    })
    .catch(err => console.error(err));
}, 50);

// === CHỨC NĂNG: RESET ===
function resetData() {
    if(confirm("Bạn có chắc muốn xóa toàn bộ dữ liệu đồ thị và dữ liệu đã lưu không?")) {
        // Xóa đồ thị
        irChart.data.labels = [];
        irChart.data.datasets[0].data = [];
        spo2Chart.data.labels = [];
        spo2Chart.data.datasets[0].data = [];
        irChart.update();
        spo2Chart.update();
        
        // Xóa dữ liệu lưu trữ
        recordedData = [];
        timeCounter = 0;
    }
}

// === CHỨC NĂNG: TẢI VỀ EXCEL/CSV ===
function downloadCSV() {
    if (recordedData.length === 0) {
        alert("Chưa có dữ liệu để tải!");
        return;
    }

    // Tạo tiêu đề cột cho file CSV
    let csvContent = "data:text/csv;charset=utf-8,";
    csvContent += "Thoi Gian (s),Nhip Tim (BPM),SpO2 (%),Tin Hieu IR (mV)\n";

    // Duyệt qua mảng dữ liệu và thêm vào chuỗi CSV
    recordedData.forEach(function(row) {
        let rowString = `${row.time},${row.bpm},${row.spo2},${row.ir}`;
        csvContent += rowString + "\n";
    });

    // Tạo link tải ảo và tự động click
    var encodedUri = encodeURI(csvContent);
    var link = document.createElement("a");
    link.setAttribute("href", encodedUri);
    
    // Tạo tên file có ngày giờ hiện tại
    let date = new Date();
    let filename = "DuLieu_Do_SucKhoe_" + date.getHours() + "h" + date.getMinutes() + ".csv";
    
    link.setAttribute("download", filename);
    document.body.appendChild(link); // Yêu cầu bởi Firefox
    link.click();
    document.body.removeChild(link);
}
</script>
</body>
</html>
)rawliteral";