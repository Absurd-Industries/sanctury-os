#!/usr/bin/env node
const fs = require('fs'), http = require('http'), url = require('url');
const DATA_FILE = './sanctuary-data.json';

http.createServer((req, res) => {
  const path = url.parse(req.url).pathname;
  res.setHeader('Access-Control-Allow-Origin', '*');
  res.setHeader('Access-Control-Allow-Methods', 'GET,POST');
  res.setHeader('Content-Type', 'application/json');
  
  if (path === '/api/save' && req.method === 'POST') {
    let body = '';
    req.on('data', chunk => body += chunk);
    req.on('end', () => { fs.writeFileSync(DATA_FILE, body); res.end('{"status":"saved"}'); });
  } else if (path === '/api/load') {
    res.end(fs.existsSync(DATA_FILE) ? fs.readFileSync(DATA_FILE) : '[]');
  } else res.end('{"error":"not found"}');
}).listen(3001, () => console.log('🐍 Sanctuary OS API running on :3001'));