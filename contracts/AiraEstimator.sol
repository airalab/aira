pragma solidity ^0.4.9;

import 'token/TokenEmission.sol';
import 'market/Market.sol';
import './AiraMarket.sol';

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
    TokenEmission public metrics;

    /**
     * @dev AIR token address
     */
    ERC20 public air = ERC20(0);

    /**
     * @dev Market metrics trading 
     */
    Market public metricsMarket; 

    function setMetricsMarket(Market _market) onlyOwner
    { metricsMarket = _market; }

    /**
     * @dev Robot liabilities trading
     */
    AiraMarket public airaMarket;

    function setAiraMarket(AiraMarket _market) onlyOwner
    { airaMarket = _market; }

    /**
     * @dev Visionary fee in 1/10 percents
     */
    uint256 public visionaryFee;

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
        uint256 _visionaryFee,
        uint256 _investorsFee,
        uint256 _metricsPrice,
        address _metrics
    ) {
        visionaryFee = _visionaryFee;
        investorsFee = _investorsFee;
        metricsPrice = _metricsPrice;
        metrics      = TokenEmission(_metrics);
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

    /**
     * @dev Withrawal visionary comission by self
     */
    function withdraw() onlyOwner {
        if (air.balanceOf(this) > totalEstimation) {
            var totalTax = air.balanceOf(this) - totalEstimation;
            var visionaryTax = totalTax * visionaryFee / (visionaryFee + investorsFee);
            air.transfer(owner, visionaryTax);
        }
    }

    /**
     * @dev Buy robot liability by metrics
     */
    function realizeMetrics(uint256 _lot) {
        var lotPrice = airaMarket.priceOf(_lot);
        if (lotPrice == 0) throw;

        var totalTax = air.balanceOf(this) - totalEstimation;
        var investorsTax = totalTax * investorsFee / (visionaryFee + investorsFee);
        var investorsBalance = totalEstimation + investorsTax;

        var senderBalance = investorsBalance * metrics.balanceOf(msg.sender) / metrics.totalSupply();
        if (senderBalance < lotPrice) throw;

        var metricsPrice  = investorsBalance / metrics.totalSupply();
        var metricsToBurn = lotPrice / metricsPrice;
        if (!metrics.transferFrom(msg.sender, this, metricsToBurn)) throw;
        metrics.burn(metricsToBurn);

        air.approve(airaMarket, lotPrice + airaMarket.taxOf(_lot));
        airaMarket.buyAt(_lot);
    }
}
