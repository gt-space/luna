import { Component, createSignal } from 'solid-js';
import {createEffect} from 'solid-js';
import {Chart,registerables, ChartConfiguration} from 'chart.js';
import 'chartjs-adapter-luxon';
import ChartStreaming from 'chartjs-plugin-streaming';
import Zoom from 'chartjs-plugin-zoom';
import { levels, plotterValues } from './PlotterView';

Chart.register(...registerables, Zoom, ChartStreaming);


const ChartComponent: Component<{id: string, index: number}> = (props) => {

    const [thisChart, setThisChart] = createSignal();

    const refreshFrequency = 5; // in Hz
    const timespan = 30; // in seconds

    const data = {
        datasets: [
          {
            label: props.id,
            data: [],
            borderColor: "#36A2EB",
            backgroundColor: "#346A8F",
            pointBackgroundColor: "#346A8F",
            pointBorderColor: "#36A2EB",
            pointHoverBackgroundColor: "#346A8F",
            pointHoverBorderColor: "#36A2EB",
          },
          {
            label: "level",
            data: [],
            borderColor: "#C53434",
            backgroundColor: "#C53434",
            pointBackgroundColor: "#C53434",
            pointBorderColor: "#C53434",
            pointHoverBackgroundColor: "#C53434",
            pointHoverBorderColor: "#C53434",
          }
        ]
    };

    function resetChartZoom() {
      (thisChart() as Chart).resetZoom();
    }

    const onRefresh = (chart: Chart) => {
        const now = Date.now();
        chart.data.datasets.forEach(async (dataset) => {
          if (dataset.label === props.id) {
            var yVal = plotterValues()[props.index];
            dataset.data.push({
              x: now,
              y: yVal
            });
          } else {
            //console.log(levels());
            if (levels().has(props.id)) {
              if (dataset.data.length == 0) {
                for (var i = 0; i < refreshFrequency*timespan; i++) {
                  dataset.data.push({
                    x: now-(i*refreshFrequency*1000),
                    y: levels().get(props.id) as number
                  })
                }
              }
              dataset.data.push({
                x: now,
                y: levels().get(props.id) as number
              });
              if (dataset.data.length > timespan*refreshFrequency) {
                dataset.data.shift();
              }
            } else {
              if (dataset.data.length != 0) {
                dataset.data = [];
              }
            }
          }
          console.log('dataset '+dataset.label+' '+dataset.data.length);
        });
    };
    const config: ChartConfiguration = {
        type: 'line',
        data: data,
        options: {
          normalized: true,
          animation: false,
          parsing: false,
          elements: {
              point:{
                  radius: 0
              }
          },
          plugins: {
            legend: {
              labels: {
                color: 'white',
                filter: item => item.text != "level"
              }
            },
            zoom: {
              zoom: {
                wheel: {
                  enabled: true,
                },
                pinch: {
                  enabled: true
                },
                mode: 'y',
              },
            }
          },
          scales: {
            x: {
              type: 'realtime',
              realtime: {
                  duration: timespan*1000,
                  refresh: 1000/refreshFrequency,
                  delay: 0,
                  frameRate: 20,
                  ttl: undefined,
                  onRefresh: onRefresh
              },
              grid: {
                color: '#545454',
                borderColor: 'white'
              },
              ticks: {
                color: 'white'
              }
            },
            y: {
              title: {
                  display: true,
                  text: 'Value',
                  color: 'white'
              },
              grid: {
                color: '#545454',
                borderColor: 'white'
              },
              ticks: {
                color: 'white'
              }
            },
          },
          interaction: {
            intersect: false
          }
        }
    };
    createEffect(async () => {
      //console.log('test', document.getElementById(props.id) as HTMLCanvasElement);
      const myChart = new Chart(document.getElementById(props.id) as HTMLCanvasElement, config);
      setThisChart(myChart);
    });
    return (
        <div>
          <button class='chart-reset-button' onClick={resetChartZoom}>Reset Zoom</button>
          <div style="display: block; margin: 0px; width: 400px;"><canvas id={props.id} class='chart-tile'></canvas></div>
        </div>
        
    );
}

export default ChartComponent;