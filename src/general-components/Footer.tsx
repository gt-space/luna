import {Component} from 'solid-js';
import { VERSION } from '../appdata';

const Footer: Component = (props) => {

  return <div class="footer">
    Fullscale GUI version {VERSION}
  </div>
}

export default Footer