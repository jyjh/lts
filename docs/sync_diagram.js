#!/usr/bin/env node
/**
 * sync_diagram.js
 *
 * Reads the Mermaid diagram from class_diagram.mmd and injects it into
 * CLASS_DIAGRAM.md between the ```mermaid and ``` markers.
 *
 * Usage:
 *   node docs/sync_diagram.js          # run from project root
 *   node docs/sync_diagram.js --check  # exit 1 if out of sync (for CI)
 */

const fs = require('fs');
const path = require('path');

const scriptDir = __dirname;
const mmdFile = path.join(scriptDir, 'class_diagram.mmd');
const mdFile = path.join(scriptDir, 'CLASS_DIAGRAM.md');

// Read source diagram
const diagram = fs.readFileSync(mmdFile, 'utf8').trim();

// Read target markdown
let md = fs.readFileSync(mdFile, 'utf8');

// Replace content between ```mermaid and the closing ```
// Handle both \n (Unix) and \r\n (Windows) line endings
const markerOpen = md.includes('```mermaid\r\n') ? '```mermaid\r\n' : '```mermaid\n';
const markerClose = '```';

const startIdx = md.indexOf(markerOpen);
if (startIdx === -1) {
    console.error('ERROR: Could not find ```mermaid block in CLASS_DIAGRAM.md');
    process.exit(1);
}

const contentStart = startIdx + markerOpen.length;
const endIdx = md.indexOf(markerClose, contentStart);
if (endIdx === -1) {
    console.error('ERROR: Could not find closing ``` for mermaid block in CLASS_DIAGRAM.md');
    process.exit(1);
}

const oldDiagram = md.slice(contentStart, endIdx);
const newMd = md.slice(0, contentStart) + diagram + '\n' + md.slice(endIdx);

// Check mode (for CI)
if (process.argv.includes('--check')) {
    if (oldDiagram.trim() === diagram) {
        console.log('✅ CLASS_DIAGRAM.md is in sync with class_diagram.mmd');
        process.exit(0);
    } else {
        console.error('❌ CLASS_DIAGRAM.md is out of sync with class_diagram.mmd');
        console.error('   Run: node docs/sync_diagram.js');
        process.exit(1);
    }
}

// Write updated file
fs.writeFileSync(mdFile, newMd, 'utf8');
console.log('✅ Synced class_diagram.mmd → CLASS_DIAGRAM.md');