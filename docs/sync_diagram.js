#!/usr/bin/env node
/**
 * sync_diagram.js
 *
 * Scans each page folder under docs/ for .mmd (Mermaid) source files
 * and renders them to .svg images using the Mermaid CLI (`mmdc`).
 *
 * Usage:
 *   node docs/sync_diagram.js          # regenerate all SVGs
 *   node docs/sync_diagram.js --check  # exit 1 if any SVG is missing or stale
 *
 * Prerequisites:
 *   npm install -g @mermaid-js/mermaid-cli
 *   (or: npx --yes @mermaid-js/mermaid-cli <input> -o <output>)
 */

const fs = require('fs');
const path = require('path');
const { execSync } = require('child_process');

const docsDir = path.join(__dirname);

// Page folders that contain .mmd diagram sources
const diagramFolders = [
  'class-diagram',
  'simulation-loop',
  'workflow',
  'data-ingestion',
];

/**
 * Resolve the mmdc command — prefer local, fall back to npx.
 */
function getMmdcCommand() {
  try {
    execSync('mmdc --version', { stdio: 'pipe' });
    return 'mmdc';
  } catch {
    return 'npx --yes @mermaid-js/mermaid-cli';
  }
}

function main() {
  const checkMode = process.argv.includes('--check');
  const mmdc = getMmdcCommand();
  let allUpToDate = true;

  for (const folder of diagramFolders) {
    const folderPath = path.join(docsDir, folder);
    if (!fs.existsSync(folderPath)) {
      console.warn(`⚠️  Folder not found: ${folder}/`);
      continue;
    }

    // Find all .mmd files in this folder
    const mmdFiles = fs.readdirSync(folderPath).filter(f => f.endsWith('.mmd'));

    for (const mmdFile of mmdFiles) {
      const mmdPath = path.join(folderPath, mmdFile);
      const svgName = mmdFile.replace(/\.mmd$/, '.svg');
      const svgPath = path.join(folderPath, svgName);

      const mmdContent = fs.readFileSync(mmdPath, 'utf8').trim();
      const mmdHash = mmdContent.length; // simple staleness check

      if (checkMode) {
        if (!fs.existsSync(svgPath)) {
          console.error(`❌ Missing SVG: ${folder}/${svgName}`);
          allUpToDate = false;
        } else {
          const svgStat = fs.statSync(svgPath);
          const mmdStat = fs.statSync(mmdPath);
          if (mmdStat.mtimeMs > svgStat.mtimeMs) {
            console.error(`❌ Stale SVG: ${folder}/${svgName} (mmd is newer)`);
            allUpToDate = false;
          } else {
            console.log(`✅ Up to date: ${folder}/${svgName}`);
          }
        }
        continue;
      }

      // Generate SVG
      console.log(`🔄 Rendering ${folder}/${mmdFile} → ${svgName}...`);
      try {
        execSync(
          `${mmdc} -i "${mmdPath}" -o "${svgPath}" --backgroundColor transparent`,
          { stdio: 'pipe', cwd: docsDir }
        );
        console.log(`✅ Generated: ${folder}/${svgName}`);
      } catch (err) {
        console.error(`❌ Failed to render ${folder}/${mmdFile}`);
        console.error(err.stderr ? err.stderr.toString() : err.message);
        process.exitCode = 1;
      }
    }
  }

  if (checkMode && !allUpToDate) {
    console.error('\nRun: node docs/sync_diagram.js');
    process.exit(1);
  }

  if (!checkMode) {
    console.log('\n✅ All diagrams synced.');
  }
}

main();