// Seving HTML File ------------------------------------------------------------------
const http = require('http');
const fs = require('fs');
http.createServer((req,res)=>{

    // Create a read stream 
    const readStream = fs.createReadStream('turtlesimjoystick.html')

    // Write a header (let the client know what type of data im expecting)
    // Status code: 200 is good status --> http status codes
    res.writeHead(200,{'Content-type': 'text/html'});

    // Pipe html to the response  
    readStream.pipe(res);
    
}).listen(9090);
//
