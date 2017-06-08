pragma solidity ^0.4.9;

import 'market/Market.sol';
import 'market/LiabilityMarket.sol';

contract AiraEstimator is Object {
    enum Mode { Estimation, Trading }

    /**
     * @dev Aira market mode
     */
    Mode public mode = Mode.Estimation;

    /**
     * @dev Estimation start time
     */
    uint256 public startTime = now;

    /**
     * @dev Estimation duration
     */
    uint256 public estimationPeriod = 2 weeks;

    modifier onlyEstimation {
        if (now - startTime > estimationPeriod) throw;
        _;
    }

    /**
     * @dev Market metrics token address
     */
    ERC20 public metrics;

    /**
     * @dev AIR token address
     */
    ERC20 public air;

    /**
     * @dev Market metrics trading 
     */
    Market public metricsMarket; 

    /**
     * @dev Robot liabilities trading
     */
    LiabilityMarket public liabilityMarket;

    /**
     * @dev Visior fee in 1/10 percents
     */
    uint256 public visiorFee;

    /**
     * @dev Total market estimation in AIR tokens
     */
    uint256 public totalEstimation = 0;

    /**
     * @dev Investors fee in 1/10 percents
     */
    uint256 public investorsFee; 

    /**
     * @dev Market metrics token price in AIR
     */
    uint256 public metricsPrice;

    /**
     * @dev Aira market estimator contract
     */
    function AiraEstimator(
        uint256 _visiorFee,
        uint256 _investorsFee,
        uint256 _metricsPrice,
        address _metrics,
        address _air,
        address _metricsMarket,
        address _liabilityMarket
    ) {
        visiorFee       = _visiorFee;
        investorsFee    = _investorsFee;
        metricsPrice    = _metricsPrice;
        metrics         = ERC20(_metrics);
        air             = ERC20(_air);
        tokenMarket     = Market(_metricsMarket);
        liabilityMarket = LiabilityMarket(_liabilityMarket);
    }

    /**
     * @dev Buy market metrics by AIR token
     */
    function buyMetrics(uint256 _count) onlyEstimation {
        var airValue = _count * metricsPrice; 
        if (!air.transferFrom(msg.sender, this, airValue))
            throw;
        if (!metrics.transfer(msg.sender, _count))
            throw;

        totalEstimation += airValue;
    }
}
