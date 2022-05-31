#!/usr/bin/node

//your code
console.log("hi")

const http = require('http')
const fs = require('fs')

const server = http.createServer((req, res) => {
  res.writeHead(200, { 'content-type': 'text/html' })
  fs.createReadStream('turtlesimjoystick.html').pipe(res)
})

server.listen(process.env.PORT || 3000)