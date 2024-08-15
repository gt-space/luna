const { execSync } = require('child_process');
const os = require('os');

const platform = os.platform();

if (platform === 'linux') {
  execSync('npm install @esbuild/linux-x64', { stdio: 'inherit' });
  execSync('npm install @tauri-apps/cli-linux-x64-gnu', { stdio: 'inherit' });
} else {
  console.log('Skipping linux-specific-package installation on', platform);
}