using UnityEngine;
using UnityEngine.UI;
using DeepseaAUV.AI;

namespace DeepseaAUV.UI
{
    /// <summary>
    /// AI 状态面板：显示模式、规划器统计、危险簇数量等
    /// </summary>
    public class AIStatusPanel : MonoBehaviour
    {
        [SerializeField] private AUVAutopilot    _autopilot;
        [SerializeField] private ObstacleMapper  _mapper;
        [SerializeField] private Text            _modeLabel;
        [SerializeField] private Text            _planLabel;
        [SerializeField] private Text            _clusterLabel;
        [SerializeField] private Text            _speedLabel;
        [SerializeField] private Text            _hintLabel;

        public void Setup(AUVAutopilot ap, ObstacleMapper mapper)
        {
            _autopilot = ap;
            _mapper = mapper;
        }

        private void Update()
        {
            if (_autopilot == null) return;

            if (_modeLabel != null)
            {
                _modeLabel.text = $"MODE: {_autopilot.Mode}";
                _modeLabel.color = _autopilot.Mode == AUVMode.Manual 
                    ? new Color(0.9f, 0.6f, 0.3f)
                    : _autopilot.Mode == AUVMode.ObstacleAvoid 
                        ? new Color(1f, 0.3f, 0.3f) 
                        : new Color(0.3f, 1f, 0.6f);
            }

            if (_planLabel != null)
            {
                _planLabel.text = $"PLAN: {_autopilot.DebugPlan}  |  {_autopilot.DebugPlanTimeMs:F1} ms  |  nodes {_autopilot.DebugNodesExpanded}";
            }

            if (_clusterLabel != null && _mapper != null)
            {
                _clusterLabel.text = $"DANGER CLUSTERS: {_mapper.Clusters.Count}";
            }

            if (_speedLabel != null)
            {
                var auv = _autopilot.GetComponent<AUVController>();
                if (auv != null)
                {
                    _speedLabel.text = $"SPD: {auv.LinearVelocity.magnitude:F2} m/s  |  DEPTH: {-auv.transform.position.y:F1} m";
                }
            }

            if (_hintLabel != null)
            {
                _hintLabel.text = "[Tab] 切换手动/自动   [WASD] 手动操纵   [QE] 升沉   [鼠标] 转向   [R] 复位";
            }
        }
    }
}
